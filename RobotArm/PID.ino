void PID(float sita_1,float sita_2){    //no T-curve
  int starttime = millis();
  digitalWrite(en_a,HIGH);
  digitalWrite(en_b,HIGH);
  while(1){
    int duration = millis() - starttime;
    //timer.update();
    //motor 1
    count_x = megaEncoderCounter.XAxisGetCount();
    axis_1.addNewSample(count_x);
    axis_1.setSetPoint(sita_1*pprx);
    u_x = axis_1.process();
    Serial.print(u_x);
    Serial.print("\t");
    if(u_x > 0){
      analogWrite(motor1_in1,u_x);
      analogWrite(motor1_in2,0);
    }
    if(u_x < 0){
      analogWrite(motor1_in1,0);
      analogWrite(motor1_in2,-u_x);
    }
    //motor 2
    count_y = megaEncoderCounter.YAxisGetCount();
    axis_2.addNewSample(count_y);
    axis_2.setSetPoint(sita_2*ppry);
    u_y = axis_2.process();
    Serial.println(u_y);
    if(u_y > 0){
      analogWrite(motor2_in3,u_y);
      analogWrite(motor2_in4,0);
    }
    if(u_y < 0){
      analogWrite(motor2_in3,0);
      analogWrite(motor2_in4,-u_y);

    }
    if(duration >= 4000){
      analogWrite(motor1_in1,0);
      analogWrite(motor1_in2,0);
      digitalWrite(en_a,LOW);
      analogWrite(motor2_in3,0);
      analogWrite(motor2_in4,0);
      digitalWrite(en_b,LOW);
      break;
    }
  }
  UserInterface();
}
