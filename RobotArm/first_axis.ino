void deg_axis_1(float degree){   //rotate axis_1
  digitalWrite(en_a,HIGH);
  while(1){
    count_x = megaEncoderCounter.XAxisGetCount();
    timer.update();
    if(degree > 0){
      analogWrite(motor1_in1,60);
      analogWrite(motor1_in2,0);
      if(count_x >= degree*pprx + lastcount_x)
        {
          analogWrite(motor1_in1,0);
          analogWrite(motor1_in2,0);
          digitalWrite(en_a,LOW);
          break;
        }
    }
    if(degree < 0){
      analogWrite(motor1_in1,0);
      analogWrite(motor1_in2,60);
      if(count_x <= degree*pprx + lastcount_x)
        {
          analogWrite(motor1_in1,0);
          analogWrite(motor1_in2,0);
          digitalWrite(en_a,LOW);
          break;
        }
    }
  }
  lastcount_x = count_x;
  UserInterface();
}
