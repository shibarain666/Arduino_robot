/* 2 kinds of mode*/
/* one is PID+T-curve, the other one is FSMC+T-curve */

static void Tcurve_initial(){
    cnt1 = (signed long)megaEncoderCounter.XAxisGetCount();
    cnt2 = (signed long)megaEncoderCounter.YAxisGetCount();
    if(sita1*pprx >= cnt1){
      X1 = sita1 - (float)cnt1/pprx;
      flg1 = 1;
    } 
    else{
      X1 = (float)cnt1/pprx - sita1;
      flg1 = 0; 
    }
   if(sita2*ppry >= cnt2){
      X2 = sita2 - (float)cnt2/ppry;
      flg2 = 1;
    }
    else{
      X2 = (float)cnt2/ppry - sita2; 
      flg2 = 0; 
    }
    float big = max(X1,X2);
    if(big == X1)
      tBiggest1();
    else if(big == X2)
      tBiggest2();
}

static void tBiggest1(){
    if(X1 >= pow(Vmax,2)/Amax){
      Ta1 = Vmax/Amax;
      Time = X1/Vmax + Ta1;
      V1 = Vmax;
      a1 = Amax;
      }
    else{
      Ta1 = sqrt(X1/Amax);
      Time = 2*Ta1;
      V1 = sqrt(X1*Amax);
      a1 = Amax;
      }
    Ta2 = Ta1 ;
    a2 = X2/(Time*Ta2-Ta2*Ta2);
    V2 = a2*Ta2;  
}
  
static void tBiggest2(){
    if(X2 >= pow(Vmax,2)/Amax){
      Ta2 = Vmax/Amax;
      Time = X2/Vmax + Ta2;
      V2 = Vmax;
      a2 = Amax;
      }
    else{
      Ta2 = sqrt(X2/Amax);
      Time = 2*Ta2;
      V2 = sqrt(X2*Amax);
      a2 = Amax;
      }
    Ta1 = Ta2 ;
    a1 = X1/(Time*Ta1-Ta1*Ta1);
    V1 = a1*Ta1; 
}

float part1(float _a){
  float a = ((_a)/2.0)*pow(((millis()-T)/1000.0),2);
  return a;
  }
  
float part2(float _a,float _maxV,float _Ta){
  float a = (_maxV)*((millis()-T)/1000.0-_Ta) + (_a)/2.0*pow(_Ta,2);
  return a;
  }
  
float part3(float _a,float _Vmax,float _Ta){
  float a =  _Vmax*(Time-_Ta) - ((_a)/2.0)*pow((millis()-T)/1000.0-Time,2);
  return a;
  }

static void T_curve(){
      Tcurve_initial();
      T = millis(); 
      digitalWrite(en_a, HIGH);
      digitalWrite(en_b, HIGH);
     
      while(1){
         timer.update();
//        Serial.print((signed long)megaEncoderCounter.XAxisGetCount()/pprx);  Serial.print("\t");  Serial.print(A /pprx); Serial.print("\t");
//        Serial.print((signed long)megaEncoderCounter.YAxisGetCount()/ppry);  Serial.print("\t");  Serial.print(B /ppry); Serial.print("\t");
//        Serial.print((millis()-T)/1000.0);
//        Serial.print("\t");
//        Serial.println(Time);

        //*Motor1 setPoint*//
        if ((millis()-T)/1000.0 <= Ta1){
          if(flg1 == 1)
          {A = cnt1 + part1(a1)*pprx;}
          else
          {A = cnt1 - part1(a1)*pprx;}
          }
        else if(Ta1 < (millis()-T)/1000.0 && (millis()-T)/1000.0<= (Time - Ta1)){
          if(flg1 == 1)
          {A = cnt1 + part2(a1,V1,Ta1)*pprx;}
          else
          {A = cnt1 - part2(a1,V1,Ta1)*pprx;}
          }
        else if((Time - Ta1) < (millis()-T)/1000.0 && (millis()-T)/1000.0 <= Time ){
          if(flg1 == 1)
          {A = cnt1 + part3(a1,V1,Ta1)*pprx;}
          else
          {A = cnt1 - part3(a1,V1,Ta1)*pprx;}
          }
        else if((millis()-T)/1000.0 > Time){
          A = sita1*pprx;
          }
        axis_1.setSetPoint(A);
            
        //*Motor1 PID*//
        axis_1.addNewSample((signed long)megaEncoderCounter.XAxisGetCount());
        float u1 = axis_1.process();
        if(u1 > 0){
          analogWrite(motor1_in1,u1);
          analogWrite(motor1_in2,0);
          }
        else if(u1 < 0){
          analogWrite(motor1_in1,0);
          analogWrite(motor1_in2,-u1);
          }
        else{
          analogWrite(motor1_in1,0);
          analogWrite(motor1_in2,0);
          }
            
        //*Motor2 setPoint*//
        if ((millis()-T)/1000.0 <= Ta2){
          if(flg2 == 1)
          {B = cnt2 + part1(a2)*ppry;}
          else
          {B = cnt2 - part1(a2)*ppry;}
          }
        else if(Ta2 < (millis()-T)/1000.0 && (millis()-T)/1000.0<= (Time - Ta2)){
          if(flg2 == 1)
          {B = cnt2 + part2(a2,V2,Ta2)*ppry;}
          else
          {B = cnt2 - part2(a2,V2,Ta2)*ppry;}
          }
        else if((Time - Ta2) < (millis()-T)/1000.0 && (millis()-T)/1000.0 <= Time ){
          if(flg2 == 1)
          {B = cnt2 + part3(a2,V2,Ta2)*ppry;}
          else
          {B = cnt2 - part3(a2,V2,Ta2)*ppry;}
          }
        else if((millis()-T)/1000.0 > Time){
          B = sita2*ppry;
          }
        axis_2.setSetPoint(B);
            
        //*Motor2 PID*//
        axis_2.addNewSample((signed long)megaEncoderCounter.YAxisGetCount());
        float u2 = axis_2.process();
        if(u2 > 0){
          analogWrite(motor2_in3,u2);
          analogWrite(motor2_in4,0);
          }
        else if(u2 < 0){
          analogWrite(motor2_in3,0);
          analogWrite(motor2_in4,-u2);
          }
        else{
          analogWrite(motor2_in3,0);
          analogWrite(motor2_in4,0);
          }
              
        if(Serial.read() == 'Q'){
            break;
            }
           if((millis()-T)/1000.0>(Time+1)){
            break;
            } 
//        Serial.println(u1);
//        Serial.println(u2);
        }
      digitalWrite(en_a, LOW);
      digitalWrite(en_b, LOW);
      UserInterface();
}

static void T_curve_fsmc(){
      Tcurve_initial();
      T = millis(); 
      digitalWrite(en_a, HIGH);
      digitalWrite(en_b, HIGH);
      while(1){
        timer.update();
//        Serial.print((signed long)megaEncoderCounter.XAxisGetCount()/pprx);  Serial.print("\t");  Serial.print(A /pprx); Serial.print("\t");
//        Serial.print((signed long)megaEncoderCounter.YAxisGetCount()/ppry);  Serial.print("\t");  Serial.print(B /ppry); Serial.print("\t");
//        Serial.print((millis()-T)/1000.0);
//        Serial.print("\t");
//        Serial.println(Time);

        //*Motor1 setPoint*//
        if ((millis()-T)/1000.0 <= Ta1){
          if(flg1 == 1)
          {A = cnt1 + part1(a1)*pprx;}
          else
          {A = cnt1 - part1(a1)*pprx;}
          }
        else if(Ta1 < (millis()-T)/1000.0 && (millis()-T)/1000.0<= (Time - Ta1)){
          if(flg1 == 1)
          {A = cnt1 + part2(a1,V1,Ta1)*pprx;}
          else
          {A = cnt1 - part2(a1,V1,Ta1)*pprx;}
          }
        else if((Time - Ta1) < (millis()-T)/1000.0 && (millis()-T)/1000.0 <= Time ){
          if(flg1 == 1)
          {A = cnt1 + part3(a1,V1,Ta1)*pprx;}
          else
          {A = cnt1 - part3(a1,V1,Ta1)*pprx;}
          }
        else if((millis()-T)/1000.0 > Time){
          A = sita1*pprx;
          }
            
        //*Motor1 FSMC*//
        count_x = megaEncoderCounter.XAxisGetCount();
        error1 = A - count_x;
        error_d1 = error1 - error_old1;
        error_old1 = error1;
        sliding_value1 = error_d1 + (0.5 * error1); //0.5是浪打，可以自己調
    
       if(abs(error1)<=1){
            u1 = fsmc(gs1[0],sliding_value1,fsmcvector1,gu1[0]);
            _gu1 = gu1[0];
       }
        else if(abs(error1)>1 && abs(error1)<=3){
            u1 = fsmc(gs1[1],sliding_value1,fsmcvector1,gu1[1]);
            _gu1 = gu1[1];
        }
        else {
           u1 = fsmc(gs1[2],sliding_value1,fsmcvector1,gu1[2]);
           _gu1 = gu1[2];
        }

        if(u1 > 0){
          analogWrite(motor1_in1,u1);
          analogWrite(motor1_in2,0);
          }
        else if(u1 < 0){
          analogWrite(motor1_in1,0);
          analogWrite(motor1_in2,-u1);
          }
        else{
          analogWrite(motor1_in1,0);
          analogWrite(motor1_in2,0);
          }
            
        //*Motor2 setPoint*//
        if ((millis()-T)/1000.0 <= Ta2){
          if(flg2 == 1)
          {B = cnt2 + part1(a2)*ppry;}
          else
          {B = cnt2 - part1(a2)*ppry;}
          }
        else if(Ta2 < (millis()-T)/1000.0 && (millis()-T)/1000.0<= (Time - Ta2)){
          if(flg2 == 1)
          {B = cnt2 + part2(a2,V2,Ta2)*ppry;}
          else
          {B = cnt2 - part2(a2,V2,Ta2)*ppry;}
          }
        else if((Time - Ta2) < (millis()-T)/1000.0 && (millis()-T)/1000.0 <= Time ){
          if(flg2 == 1)
          {B = cnt2 + part3(a2,V2,Ta2)*ppry;}
          else
          {B = cnt2 - part3(a2,V2,Ta2)*ppry;}
          }
        else if((millis()-T)/1000.0 > Time){
          B = sita2*ppry;
          }
            
        //*Motor2 FSMC*//
        count_y = megaEncoderCounter.YAxisGetCount();
        error2 = B - count_y;
        error_d2 = error2 - error_old2;
        error_old2 = error2;
        sliding_value2 = error_d2 + (0.5 * error2);

        if(abs(error2)<=1){
            u2 = fsmc(gs2[0],sliding_value2,fsmcvector2,gu2[0]);
            _gu2 = gu2[0];
        }
        else if(abs(error1)>1 && abs(error1)<=3){
            u2 = fsmc(gs2[1],sliding_value2,fsmcvector2,gu2[1]);
            _gu2 = gu2[1];
        }
        else {
           u2 = fsmc(gs2[2],sliding_value2,fsmcvector2,gu2[2]);
           _gu2 = gu2[2];
        }
        
        if(u2 > 0){
          analogWrite(motor2_in3,u2);
          analogWrite(motor2_in4,0);
          }
        else if(u2 < 0){
          analogWrite(motor2_in3,0);
          analogWrite(motor2_in4,-u2);
          }
        else{
          analogWrite(motor2_in3,0);
          analogWrite(motor2_in4,0);
          }
              
        if(Serial.read() == 'Q'){
            break;
            }
           if((millis()-T)/1000.0>(Time+1)){
            break;
            } 
//        Serial.println(u1);
//        Serial.println(u2);
        }
      digitalWrite(en_a, LOW);
      digitalWrite(en_b, LOW);
      UserInterface();
}
