void Calibration(){
    int val,val2;
    bool cali_1 = 0, cali_2 = 0;
    detachInterrupt(digitalPinToInterrupt(2));
    digitalWrite(en_a,HIGH);
    digitalWrite(en_b,HIGH);
    analogWrite(motor1_in1,100);  //先正轉
    analogWrite(motor1_in2,0);
    analogWrite(motor2_in3,20);   //先正轉
    analogWrite(motor2_in4,0);
    while(1){
      val = digitalRead(2); 
      val2 = digitalRead(3); 
      count_x = megaEncoderCounter.XAxisGetCount();
      count_y = megaEncoderCounter.YAxisGetCount();
      timer.update();
      if(val == 1){
        megaEncoderCounter.XAxisReset( );
        analogWrite(motor1_in1,0);
        analogWrite(motor1_in2,80);
      }  
      if(val2 == 1){
        megaEncoderCounter.YAxisReset( );
        analogWrite(motor2_in3,0);
        analogWrite(motor2_in4,20);
      }
      if(count_x <= -90*pprx)   
      {
        analogWrite(motor1_in1,0);
        analogWrite(motor1_in2,0);
        cali_1 = 1;
      }
      if(count_y <= -180*ppry)    
      {
        analogWrite(motor2_in3,0);
        analogWrite(motor2_in4,0);
        cali_2 = 1;
      }
      if(cali_1 == 1 && cali_2 == 1)
      {
        analogWrite(motor1_in1,0);
        analogWrite(motor1_in2,0);
        analogWrite(motor2_in3,0); 
        analogWrite(motor2_in4,0);
        break;
      }
    }
    delay(1000);
    megaEncoderCounter.XAxisReset( );
    megaEncoderCounter.YAxisReset( );
    attachInterrupt(digitalPinToInterrupt(2),limit, HIGH);
    UserInterface();
}
