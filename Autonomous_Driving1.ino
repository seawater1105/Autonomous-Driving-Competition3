#include <RPi_Pico_TimerInterrupt.h>
int vd = 0, wd = 0; 
int cl, cr; 
int v = 80;
int v1 = 90;
float A = 5.6;
int Kpv = 10, Kdv = 2;
float Kpw = 6.6, Kdw = 3; 
int errVp = 0;
int errWp = 0;
int ex_stk = 0; //탈출 기능 스택
RPI_PICO_Timer ITimer0(0);
int gap;
int forw = 0;
int right2 = 100, left2 = 100; //자동차 바로 양 옆
int x[520], y[520];
uint8_t cnl, cnr;
uint8_t cnl2, cnr2;
uint8_t cnl3, cnr3;
int right, left; //전방 좌우 구분  정상주행할때 쓰임
int l1, l2, r1, r2;
int g1, g2;
int b; //레이더 감지 범위 이상해서 넣은 함수
bool chk=0;
int right_stk = 0, left_stk = 0; //탈출이후 방향 정하는 변수

bool TimerHandler0(struct repeating_timer *t)
{ 
  int vl = cl, vr = cr;
    cl = 0; cr = 0;
  int V = (vl+vr);
  int W = (vl-vr)*0.13;

  int errV = (vd-V);
  int dv = errV-errVp;
  int errW = (wd-W);
  int dw = errW-errWp;

  int mv = A*vd + Kpv*(vd - V) + Kdv*dv ;
  int mw = Kpw*(wd - W) + Kdw*dw;
  motor(mv+mw,mv-mw);
  
  errVp= errV;
  errWp= errW;

  return true; }



void setup() {
  pinMode(29,INPUT);
  ITimer0.attachInterruptInterval(10 * 1000, TimerHandler0);

  pinMode(14,INPUT); pinMode(15,INPUT);
  // 14,15 이 왼쪽 모터의 엔코더 A,B상임.
  attachInterrupt(digitalPinToInterrupt(14), L1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(15), L2, CHANGE);
  // 14번이 (CHANGE or FALLING or RISING)되면 L1함수를 실행시킨다. 
  
  pinMode(2,INPUT); pinMode(3,INPUT);
  // 2,3 이 오른쪽 모터
  attachInterrupt(digitalPinToInterrupt(2), R1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), R2, CHANGE);
  analogWriteResolution(10);
  // PWM 신호를 Bit값으로 설정 
  // 10으로 하면 0~1023 (밑에 모터 함수에서 -1023~1023 까지로 범위를 정했기 때문.)


 pinMode(27,OUTPUT); pinMode(28,OUTPUT);
Serial.begin(500000);
Serial1.begin(460800);
Serial1.setRX(1); Serial1.setTX(0);   
delay(2000); 


Serial1.write(0xA5); Serial1.write(0x20);
byte recv[7];  int p=0;
while(!Serial1.available()) {delay(1);}
 
while( (Serial1.available()) || (p<7) )
{ recv[p]=Serial1.read(); p++; delay(1);  }

multicore_launch_core1(main2);

}


void loop() {
  int sensor = analogRead(29);
  

  if (chk==1) {  bool lef=cnl>0, rig=cnr>0; chk=0;  bool lef2=cnl2>0, rig2=cnr2>0; chk=0; bool lef3=cnl3>0, rig3=cnr3>0;
                        g1 = l1 + r1;
                        g2 = l2 + r2;
                        int a = abs(g1-g2);
                        
          if (sensor>30){
                if(ex_stk == 0 )  
                {
                  if ( !digitalRead(28) && !digitalRead(27) )  //정상주행
                  { vd = 40; wd = 0; chk=1;    }

                  if(forw > 280)
                  {
                         if ( digitalRead(28) && !digitalRead(27) ) 
                         {vd = 5; wd = 70; chk=1;   }
                         if ( !digitalRead(28) && digitalRead(27) ) 
                         {vd = 5; wd = -70; chk=1;  } 
                  }
                  
                   if(digitalRead(28)&&digitalRead(27)) { 
                     gap = abs(left - right);
                                         if (gap <90) 
                                         {
                                          
                                          //if(forw > 65) {
                                           // vd=10; wd=0; chk=1;
                                         // }
                                         // else{
                                           
                                            if(forw>135)
                                                 {
                                                  vd = 20; wd=0; chk=1;
                                                 }
                                             else{
                                                    if(a < 5)
                                                    {
                                                      
                                                      if(rig2&&lef2)
                                                       { 
                                                        //if(ex_stk ==0){
                                                        //vd = 5; wd=-70; 
                                                        //}
                                                          ex_stk = 1;
                                                         // vd=0; wd=-80;
                                                          chk=1;
                                                      
                                                    }
                                             }
                                          }
                                          
                                         }
                     
                          else{
                               if(left>right) 
                               { 
                                
                                vd = 5; wd = -v1; chk=1;
                                
                               } 
                                
                               
                               if(left<right) 
                               {
                                
                                vd = 5; wd = v1; chk=1;
                                
                               }
                      
                     
                          }
                     
                   
                     
                   }
                  
                }
                else 
                { //탈출 시작

                      
                       
                          if(!left_stk&&!right_stk)
                          {
                            gap = abs(l2 - r2);
                            if (gap >130)
                            {
                              if(l2 > r2)
                              {
                                left_stk = 1; chk = 1;
                              }
                              if(r2 > l2)
                              {
                                right_stk = 1; chk = 1;
                              }
                            }
                            else{
                                if(!lef3&&!rig3){
                                 vd = -10; wd = 0; chk = 1; //후진
                                }
                                 else {
                                  vd = 40;
                                 }
                            }
                           
                          }
                          else
                          {
                            if(left_stk == 1)
                            {
                              if(!digitalRead(28) && digitalRead(27))
                              {
                                left_stk = 0; ex_stk = 0; chk = 1;
                              }
                              else {
                                
                                if(!rig3&&!lef3){
                                    vd = 0; wd = -75; chk= 1;
                                  
                                 }
                                 else{
                                  vd = 40; wd = 0; chk =1;
                                  
                                 }
                              }
                            }
                            if(right_stk == 1)
                            {
                              if(!digitalRead(28) &&  digitalRead(27))
                              {
                                right_stk = 0; ex_stk = 0; chk = 1;
                              }
                              else {
                                
                                
                                if(!rig3&&!lef3){
                                 vd = 0; wd = 75; chk = 1;
                                  
                                 }
                                 else{
                                  vd = 40; wd =0; chk=1;
                                 }
                              }
                            }
                            
                          
                  }
                }
             
                 digitalWrite(28,lef); digitalWrite(27,rig); 
    }
    else{
      vd=0; wd=0; chk=1;
    }
  }
  else {delayMicroseconds(50);   }
  
}


 
void main2()
{
  unsigned int ps=0;
  uint8_t n=0, cntl=0,cntr=0;
  uint8_t cntl2=0,cntr2=0;
  uint8_t cntl3=0,cntr3=0;
  bool upd=0;
  byte d[5];


  
while(1) {
if (Serial1.available()) 
  { d[n]=Serial1.read();   n++; 
    if (n>4) 
      {  n=0;  
         if (bitRead(d[0],0)==1)  {  ps=0; upd=0; }
         else {ps++;}
         unsigned int ang=(d[1]>>1)+(d[2]<<7) , dis=(d[3]+(d[4]<<8))>>2;
         if  (dis>0) {   
           float q=(ang*PI)/11520;
           int tempx=sin(q)*dis,  tempy=cos(q)*dis; 
           x[ps]=tempx; y[ps]=tempy;
           if ( (abs(tempx)<90) && (tempy>90) && (tempy<250)  ) 
                 { if (tempx<0) {cntl++;} else {cntr++;}  }    
           if ( (abs(tempx)<100) && (tempy>-135) && (tempy<10)  ) 
                 { if (tempx<0) {cntl3++;} else {cntr3++;}  }  
//           if ( (abs(tempx)<100) && (tempy>-10) && (tempy<0)  ) 
//                 { if (tempx<0) {cntl3++;} else {cntr3++;}  } 
            if ((tempy < 105) && (tempy > 90)) //y 범위에서 오른쪽 왼쪽 값 거리를 저장
            {
              if (tempx>=0) {
                if(tempx==0){
                  tempx = 999;
                }
                   right = tempx;
              }
              else {
                   
                   left = abs(tempx);
              }
            }
            if ((tempy < 11) && (tempy > 9)) //y 범위에서 오른쪽 왼쪽 값 거리를 저장
            {
              if (tempx>=0) {
                if(tempx==0){
                  tempx = 999;
                }
                   r1 = tempx;
              }
              else {
                   
                   l1 = abs(tempx);
              }
            }
            if ((tempy < -9) && (tempy > -11)) //y 범위에서 오른쪽 왼쪽 값 거리를 저장
            {
              if (tempx>=0) {
                if(tempx==0){
                  tempx = 999;
                }
                   r2 = tempx;
              }
              else {
                   
                   l2 = abs(tempx);
              }
            }
            if ((abs(tempx)<2)&&(tempy < 500)&&(tempy > 10)) //전방거리 측정
            {
              forw = tempy;
            }
//           if(tempy>20){
//             if((tempx>100)&&(tempx<210))
//             {
//               b = tempx;
//             }
//             else if ((tempx >0)&&(tempx<100))
//             {
//              b = 1000;
//             }
//           }
            if ((abs(tempx)<200) && (tempy>10) && (tempy<20)) // 폭 보다 조금 넓게 봄.
            { 
              if (tempx>=0) 
            {cntr2++;} 
            else {cntl2++;} 
            
            
            }
                 
           }
     
           
          
          if ( (ang> 5760) &&(upd==0)  ) {cnl=cntl; cnr=cntr; cntl=0; cntr=0; cnl2=cntl2; cnr2=cntr2; cntl2=0; cntr2=0; cnl3=cntl3; cnr3=cntr3; cntl3=0; cntr3=0; chk=1; upd=1;}
      }
  }
 else  {delayMicroseconds(10);     }    
 
}

}
  
 void L1()
{
  if(digitalRead(14)!=digitalRead(15)) // "왼쪽 모터의 엔코더가 반대로 연결되어있음."
    {cl++;}
  // 14가 B 15가 A라고 생각하고 B상 인터럽트가 걸렸을때
  // A상 B상의 값이 다르므로 정방향이다 -> cr++ 해준다.
  else
    {cl--;}
  // B상 인터럽트가 걸렸을때 A상 B상의 값이 같으므로 역방향이다 -> cr--
}
void L2()
{
  if(digitalRead(14)==digitalRead(15))
    {cl++;}
  // A상 인터럽트가 걸렸을때, A상 B상의 값이 같으므로 -> cr++ 
  else
    {cl--;}
  // B상 인터럽트가 걸렸을때, A상 B상의 값이 다르므로 -> cr--
}

void R1() 
{
  if(digitalRead(2)==digitalRead(3))
    {cr++;}
  // 2가 A 3이 B라고 생각하면 A상 인터럽트가 걸렸을때,
  // A상 B상의 값이 같으면 정방향이다 -> cr++를 해준다.
  else
    {cr--;}
  // A상 인터럽트가 걸렸을때, A상 B상 값이 다르므로 역방향이다 -> cr--
}
void R2()
{
  if(digitalRead(2)!=digitalRead(3))
    {cr++;} 
  // B상 인터럽트가 걸렸을때, A상 B상의 값이 다르면 정방향이다 -> cr++
  else
     {cr--;}
  // B상 인터럽트가 걸렸을때, A상 B상의 값이 같으므로 역방향이다 -> cr--
}


void motor(int vl, int vr)
{
  vl=constrain(vl, -1023, 1023);
  vr=constrain(vr, -1023, 1023);
  if(vl>0){analogWrite(12,0); analogWrite(13,vl);}
  else {analogWrite(13,0); analogWrite(12,-vl);}

  if(vr>0){analogWrite(10,0); analogWrite(11,vr);}
  else {analogWrite(11,0); analogWrite(10,-vr);}

}