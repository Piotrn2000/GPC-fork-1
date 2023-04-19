#include <Wire.h>
#include <elapsedMillis.h>
#include "konfig.h"
#include "matrix.h"
#include "mpc.h"

//obsługa karty SD
#include <SD.h>
#include <SPI.h>

File myFile;

const int chipSelect = BUILTIN_SDCARD;

int steps = 0;
//Deklaracja macierzy
Matrix CA(HY, HY);
Matrix CB(HY, HU);
Matrix HA(HY, NA);
Matrix HB(HY, NB);

Matrix H(HY, HU);
Matrix P1(HY, NA);
Matrix P2(HY,NB);

Matrix W(HU, HU);
Matrix I(HU, HU);
Matrix jed(1 ,HU);
Matrix pomocnicza(1, HU);

Matrix YR(HY, 1);  //zmiana wartości od potencjomnetru 
Matrix Ywlewo(NA, 1);
Matrix deltaUwlewo(NB, 1);
Matrix V(HU, 1);

//delta u przewidujace
Matrix deltaUwprawo(HU, 1);

Matrix deltaUk(1, 1);

Matrix UK(1, 1);
Matrix UKminusjeden(1, 1);

Matrix E(0,0);  /* ograniczenia EX<=F */
Matrix F(0,0);  /* =Matrix(0,0) ? */

/* vCreateConstrains LHS/RHS ? */

//wartość zadana
int wz = 20;

//chwilowe wyjscie
Matrix wyjscie(1, 1);

Matrix macierzdoprzesuwania(HU, HU);
void setup() {
    /* serial to display data */
    Serial.begin(115200);
    while(!Serial) {}
    pinMode(LED_BUILTIN, OUTPUT);
    
    //inicjalizacja karty SD
    Serial.print("Inicjalizacja karty SD: ");

    if (!SD.begin(chipSelect)) {
      Serial.println("Nieudana");
      return;
    }
    Serial.println("Udana.");


    
    //Przypisanie wartości do macierzy CA z karty SD

    readmatrix("CA.txt", CA, HY, HY);
    
    //Przypisanie wartości do macierzy CB z karty SD
    
    readmatrix("CB.txt", CB, HY, HU);
    
    //Przypisanie wartości do macierzy HA z karty SD
    
    readmatrix("HA.txt", HA, HY, NA);

    //Przypisanie wartości do macierzy HB z karty SD
    
    readmatrix("HB.txt", HB, HY, NB);

    //Przypisanie wartości do macierzy I z karty SD
    
    readmatrix("I.txt", I, HU, HU);

    //Przypisanie wartości do macierzy I z karty SD
    
    readmatrix("jed.txt", jed, 1, HU);

    //Obliczanie macierzy H, P1, P2
    
    H = CA.Invers() * CB;
    P1 = -1* CA.Invers() * HA;
    P2 =  CA.Invers() * HB;
    
    //Wyświetlanie macierzy P2
    
    //P2.vPrintFull();
    float p = 0.5;
    W = 2*(H.Transpose()*H+p*I);
    pomocnicza = -1*jed*W.Invers();

    //V = -2*H.Transpose()*(YR-P1*Ywlewo-P2*deltaUwlewo); //loop dodane
    //deltaUk = pomocnicza*V; //loop dodane
    
    //deltaUwprawo[0][0]= deltaUk[0][0];
    
    //generowanie macierzy do przesuwania
    for(int i = 0; i<HU-1; i++ ){
      macierzdoprzesuwania[i][i+1]=1;
      
    }
    //generowanie macierzy YR
       for(int i = 0; i<HU; i++ ){
      YR[i][0]=wz;
      
    }
      
    //YR = YR.Transpose();
    //macierzdoprzesuwania[HY-1][0]=1;
   
    //YR = YR*macierzdoprzesuwania;
    //YR = YR.Transpose();

    
    /*//przesuwanie macierzy Ywlewo
    for(int i = NA-1; i>0; i-- )
    { 
      Ywlewo[i][0]=Ywlewo[i-1][0];
      }
     Ywlewo[0][0]=wyjscie[0][0];*/

    
    
    //wyliczanie delta u w lewo deltaUwlewo trzeba dopasować do transmitancji 
    //deltaUwlewo[0][0] =deltaUk[0][0];
    
    //Ywlewo.vPrintFull();

    //obliczanie chwilowego wyjscia - to na końcu
    //wyjscie=H*deltaUwprawo+P1*Ywlewo+P2*deltaUwlewo;

    
   }
  

void loop() {
  //loop
  if(steps < 40)
  {
  V = -2*H.Transpose()*(YR-P1*Ywlewo-P2*deltaUwlewo); //obliczanie wartosci V
  /*deltaUk = pomocnicza*V; //obliczanie wartosci delta Uk */
  if (!bActiveSet(deltaUk, W, V, E, F)) deltaUk.vSetToZero();	/* Quadprog, Ex<=F , wymiary: E - ilość ograniczeń na wysokość deltaUk, F - ilość ograniczeń na 1 */

  
  deltaUwprawo[0][0]= deltaUk[0][0]; //zweryfikowc bo wpisujemy tylko pierwsza wartosc, reszta to zera
  
  wyjscie=H*deltaUwprawo+P1*Ywlewo+P2*deltaUwlewo; //obliczanie chwilowego wyjscia

  //przesuwanie
  //przesuwanie macierzy Ywlewo
    for(int i = NA-1; i>0; i-- )
    { 
      Ywlewo[i][0]=Ywlewo[i-1][0];
      }
     Ywlewo[0][0]=wyjscie[0][0];
   
   //wyliczanie delta u w lewo deltaUwlewo trzeba dopasować do transmitancji 
    deltaUwlewo[0][0] =deltaUk[0][0];
    Serial.println(wyjscie[0][0]);
    //wyjscie.vPrintFull();
    delay(1000);
    steps = steps + 1;
  }
}


void readmatrix(char const *filename, Matrix &matrix, int x, int y)
{
     myFile = SD.open(filename);

    if (myFile) 
  {
    while (myFile.available())
    {  
      for(int i=0;i<x;i++)
      {
        
        for(int j=0;j<y;j++)
        {
          float myFloat = myFile.parseFloat();
          matrix[i][j]=myFloat;
        }
      }
      myFile.close();  
    }
  }  
  else {
    Serial.println("Nie można otworzyć pliku: " + String(filename));
   
  }       
 
}



void SPEW_THE_ERROR(char const * str)
{
  //Funkcja potrzebna do działania biblioteki mpc.h/matrix.h
  
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}

bool MPC::bActiveSet(Matrix &x, const Matrix &Q, const Matrix &c, const Matrix &ineqLHS, const Matrix &ineqRHS)
{
    bool _flagConstActive[ineqRHS.i16getRow()];    /* Contains information about which inequality is active (if true, then that inequality is active). */
    bool _dlambdaPos, _dxNol;
    Matrix dx(x.i16getRow(), 1, Matrix::NoInitMatZero);
    
    
    /* TODO: Make sure x initial value is inside feasible region (e.g. using Linear Programming).
     *       For now we set it to zero --> no mathematical guarantee!
     */
    x.vSetToZero();
    /* In the beginning, every constraints is non-active */
    for (int16_t _i = 0; _i < ineqRHS.i16getRow(); _i++) {
        _flagConstActive[_i] = false;
    }
    
    
    int16_t _i16iterActiveSet = 0;  /* Reset counter iteration Active Set */
    do {
        /* Construct active set matrix Aw (_ineqActiveLHS) */
        uint16_t _u16cntConstActive = 0;
        for (int16_t _i = 0; _i < ineqLHS.i16getRow(); _i++) {
            if (_flagConstActive[_i] == true) {
                _u16cntConstActive++;
            }
        }
        
        /* Construct Active Set solution matrix (the KKT matrix):
         * 
         *  _KKT_LHS(k) * [  dx   ] = _KKT_RHS(k)
         *                [dLambda]
         * 
         *  _KKT_LHS(k) = [        Q           _ineqActiveLHS(k)']
         *                [_ineqActiveLHS(k)           0         ]
         * 
         *  _KKT_RHS(k) = [-(Q*x+c)]
         *                [    0   ]
         */
        int16_t _indexConstActive[_u16cntConstActive];                                          /* Contains information about index of the active inequality constraints in ineqLHS */
        Matrix _ineqActiveLHS(_u16cntConstActive, ineqLHS.i16getCol(), Matrix::NoInitMatZero);  /* Aggregation of the row vectors of ineqLHS that is 'active', i.e. the Aw matrix */
        
        for (int16_t _i = 0, _iterConstActive = 0; _i < ineqLHS.i16getRow(); _i++) {
            if (_flagConstActive[_i] == true) {
                _ineqActiveLHS = _ineqActiveLHS.InsertSubMatrix(ineqLHS, _iterConstActive, 0, _i, 0, 1, ineqLHS.i16getCol());
                _indexConstActive[_iterConstActive] = _i;

                _iterConstActive++;
                ASSERT((_iterConstActive <= _u16cntConstActive), "Bug on the active set: Create _ineqActiveLHS");
            }
        }
        Matrix _KKT_LHS((Q.i16getRow()+_u16cntConstActive), (Q.i16getCol()+_u16cntConstActive));
        Matrix _KKT_RHS((Q.i16getRow()+_u16cntConstActive), 1);
        
        _KKT_LHS = _KKT_LHS.InsertSubMatrix(Q, 0, 0);
        _KKT_LHS = _KKT_LHS.InsertSubMatrix(_ineqActiveLHS, Q.i16getRow(), 0);
        _KKT_LHS = _KKT_LHS.InsertSubMatrix(_ineqActiveLHS.Transpose(), 0, Q.i16getCol());
        _KKT_RHS = _KKT_RHS.InsertSubMatrix(-((Q*x)+c), 0, 0);
        
        /* [   dx  ] = [        Q           _ineqActiveLHS(k)']^-1 * [-(Q*x+c)]
         * [dLambda]   [_ineqActiveLHS(k)           0         ]      [    0   ]
         * 
         */
        Matrix _KKTvector(_KKT_LHS.Invers()*_KKT_RHS);
        if (!_KKTvector.bMatrixIsValid()) {
            return false;
        }
        dx = dx.InsertSubMatrix(_KKTvector, 0, 0, x.i16getRow(), 1);
        
        Matrix dLambda(_u16cntConstActive, 1, Matrix::NoInitMatZero);
        dLambda = dLambda.InsertSubMatrix(_KKTvector, 0, 0, x.i16getRow(), 0, _u16cntConstActive, 1);
        
        
        /* Check for Karush–Kuhn–Tucker conditions ------------------------------------------------------------------------------------------------ */
        /* Karush–Kuhn–Tucker conditions:
         *  1. dx == 0
         *  2. dLambda >= 0
         */
        /* Search for dx == 0 ----------------- */
        _dxNol = true;
        for (int16_t _i = 0; _i < Q.i16getRow(); _i++) {
            if (fabs(dx[_i][0]) > float_prec(float_prec_ZERO_ECO)) {
                _dxNol = false;
                break;
            }
        }
        /* Search for dLambda >= 0 ------------ */
        float_prec _lowestLambda = 0.0;
        int16_t _idxIneqLowestLambda = -1;
        _dlambdaPos = true;
        for (int16_t _i = 0; _i < _u16cntConstActive; _i++) {
            if (dLambda[_i][0] < _lowestLambda) {
                /* Some constraints become inactive, search constraint with lowest Lagrange multiplier so we can remove it */
                _lowestLambda = dLambda[_i][0];
                _idxIneqLowestLambda = _indexConstActive[_i];
                _dlambdaPos = false;
            }
        }
        /* Check it! -------------------------- */
        if (_dxNol && _dlambdaPos) {
            break;
        } 
        
        
        if (!_dlambdaPos) {
            /* There is a constraint that become inactive ----------------------------------------------------------------------------------------- */

            /* remove the constraint that have lowest Lagrange multiplier */
            ASSERT((_idxIneqLowestLambda != -1), "Bug on the active set: remove ineq active");
            _flagConstActive[_idxIneqLowestLambda] = false;
        }
        
        
        if (!_dxNol) {
            /* It means the x(k+1) will move ------------------------------------------------------------------------------------------------------ */
            
            /* Search for violated constraints. If any, then get the lowest scalar value _alpha that makes 
             *  the searching direction feasible again
             */
            float_prec _alphaMin = 1.0;
            int16_t _idxAlphaMin = -1;
            
            Matrix _ineqNonActiveLHS_dx(ineqLHS * dx);
            Matrix _ineqNonActiveLHS_x(ineqLHS * x);
            
            for (int16_t _i = 0; _i < ineqLHS.i16getRow(); _i++) {
                if (_flagConstActive[_i] == false) {
                    /* For all constraints that is not active, we check if that constraint is violated */
                    if (_ineqNonActiveLHS_dx[_i][0] > float_prec_ZERO_ECO) {
                        /* We have a violated constraint. Make that constraint a candidate of active constraint */
                        float_prec _alphaCandidate = - (_ineqNonActiveLHS_x[_i][0] - ineqRHS[_i][0]) / _ineqNonActiveLHS_dx[_i][0];
                        
                        if (_alphaCandidate < _alphaMin) {
                            _alphaMin = _alphaCandidate;
                            _idxAlphaMin = _i;
                        }
                    }
                }
            }
            if (_idxAlphaMin != -1) {
                _flagConstActive[_idxAlphaMin] = true;
            }
            
            /* Should we do ASSERT(_alphaMin > 0.0) here? */
            if (_alphaMin > 0) {
                x = x + (dx * _alphaMin);
            }
        }
        
        
        /* Check for maximum iteration conditions ------------------------------------------------------------------------------------------------- */
        _i16iterActiveSet++;
        if (_i16iterActiveSet > MPC_MAXIMUM_ACTIVE_SET_ITERATION) {
            /* Maximum number of iteration reached, terminate and use last solution */
            break;
        }
    } while (1);
    
    return true;
}

