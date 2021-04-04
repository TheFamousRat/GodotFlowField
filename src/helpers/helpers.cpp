#include "helpers.h"

std::string generateUUID(){
  std::string uuid = std::string(36,' ');
  int rnd = 0;
  int r = 0;
  
  uuid[8] = '-';
  uuid[13] = '-';
  uuid[18] = '-';
  uuid[23] = '-';

  uuid[14] = '4';

  for(int i=0;i<36;i++){
    if (i != 8 && i != 13 && i != 18 && i != 14 && i != 23) {
      if (rnd <= 0x02) {
          rnd = 0x2000000 + (std::rand() * 0x1000000) | 0;
      }
      rnd >>= 4;
      uuid[i] = CHARS[(i == 19) ? ((rnd & 0xf) & 0x3) | 0x8 : rnd & 0xf];
    }
  }
  return uuid;
}

godot::Color HSVtoRGB(float fH, float fS, float fV) {
    float fR; 
    float fG; 
    float fB;
    float fC = fV * fS; // Chroma
    float fHPrime = fmod(fH / 60.0, 6);
    float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
    float fM = fV - fC;
    
    if(0 <= fHPrime && fHPrime < 1) {
        fR = fC;
        fG = fX;
        fB = 0;
    } else if(1 <= fHPrime && fHPrime < 2) {
        fR = fX;
        fG = fC;
        fB = 0;
    } else if(2 <= fHPrime && fHPrime < 3) {
        fR = 0;
        fG = fC;
        fB = fX;
    } else if(3 <= fHPrime && fHPrime < 4) {
        fR = 0;
        fG = fX;
        fB = fC;
    } else if(4 <= fHPrime && fHPrime < 5) {
        fR = fX;
        fG = 0;
        fB = fC;
    } else if(5 <= fHPrime && fHPrime < 6) {
        fR = fC;
        fG = 0;
        fB = fX;
    } else {
        fR = 0;
        fG = 0;
        fB = 0;
    }
    
    fR += fM;
    fG += fM;
    fB += fM;
    
    return godot::Color(fR, fG, fB, 1.0);
}