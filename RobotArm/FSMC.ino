float fsmc(float s,float x, float *v,float gain){
  // x:sliding value, s:gs, gain:gu
  // fuzzify
  //我們打的是一維，11個區間
  
  int i;
  for ( i = 0; i<11; i++)
    v[i] = 0.0;

  if (x >= 1.0*s)
    v[10] = 1.0;
  else if (x >= 0.8*s && x <= 1.0*s)         //5, -5是斜率，就是區間的三角形
  {
    v[10] = 5.0 / s*x - 4.0;
    v[9] = -5.0 / s*x + 5.0;
  }
  else if (x >= 0.6*s && x <= 0.8*s)
  {
    v[9] = 5.0 / s*x - 3.0;
    v[8] = -5.0 / s*x + 4.0;
  }
  else if (x >= 0.4*s && x <= 0.6*s)
  {
    v[8] = 5.0 / s*x - 2.0;
    v[7] = -5.0 / s*x + 3.0;
  }
  else if (x >= 0.2*s && x <= 0.4*s)
  {
    v[7] = 5.0 / s*x - 1.0;
    v[6] = -5.0 / s*x + 2.0;
  }
  else if (x >= 0.0*s && x <= 0.2*s)
  {
    v[6] = 5.0 / s*x - 0.0;
    v[5] = -5.0 / s*x + 1.0;
  }
  else if (x >= -0.2*s && x <= 0.0*s)
  {
    v[5] = 5.0 / s*x + 1.0;
    v[4] = -5.0 / s*x - 0.0;
  }
  else if (x >= -0.4*s && x <= -0.2*s)
  {
    v[4] = 5.0 / s*x + 2.0;
    v[3] = -5.0 / s*x - 1.0;
  }
  else if (x >= -0.6*s && x <= -0.4*s)
  {
    v[3] = 5.0 / s*x + 3.0;
    v[2] = -5.0 / s*x - 2.0;
  }
  else if (x >= -0.8*s && x <= -0.6*s)
  {
    v[2] = 5.0 / s*x + 4.0;
    v[1] = -5.0 / s*x - 3.0;
  }
  
  else if (x >= -1.0*s && x <= -0.8*s)
  {
    v[1] = 5.0 / s*x + 5.0;
    v[0] = -5.0 / s*x - 4.0;
  }
  else if (x <= -1.0*s)
    v[0] = 1.0;

  //sliding mode control defuzzy  
  float rule[11] = { -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 };
  float u;
  float weight = 0.0, weight_sum = 0.0;
  

  for (i = 0; i<11; i++){
    if (v[i]>0.0){
      weight += v[i];
      weight_sum += (v[i])*rule[i];
    }
  }
 
  u = (weight_sum)*gain;
  
  return u;
}
