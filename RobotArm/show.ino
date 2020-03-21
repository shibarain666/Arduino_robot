void show() {
  Serial.print("count_x : ");
  Serial.print(count_x);
  Serial.print("\t");
  Serial.print("count_y : ");
  Serial.println(count_y);
  
  //********************//
  
  //Serial.print(count_x);Serial.print("\t");Serial.print(A);Serial.print("\t");
  //Serial.print(count_y);Serial.print("\t");Serial.print(B);Serial.println("\t");
  
  //********************//
  
  //Serial.print(u1);Serial.print("\t");Serial.println(sliding_value1);
}

void limit(){ 
  analogWrite(motor1_in1,0);
  analogWrite(motor1_in2,0);
  digitalWrite(en_a,LOW);
}
