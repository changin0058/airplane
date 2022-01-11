String bleName = "air1234"; //바꾸고 싶은 이름으로 변경

void setup() {
  Serial.begin(9600); //USB 시리얼 포트

  //--------블루투스 설정----------------
  SlaveSet();  //블루투스 슬레이브로 설정
  //MasterSet(); //블루투스 마스터로 설정
}


void loop() {
    Serial.println("ok");
    delay(1000);
}

void SlaveSet(){
    Serial1.begin(115200); //통신속도 115200
    delay(1000);
    Serial1.write("AT+RENEW"); //설정 초기화
    delay(2000);
    Serial1.write("AT+RESET"); //리셋
    delay(1000);

    Serial1.begin(9600); //통신속도 9600
    delay(1000);
    Serial1.write("AT+RENEW"); //설정 초기화
    delay(2000);
    Serial1.write("AT+RESET"); //리셋
    delay(1000);

    Serial1.write("AT"); //AT
    delay(1000);
    Serial1.write("AT+BAUD4"); //통신속도 115200으로 변경
    delay(1000);
    Serial1.write("AT+RESET"); //리셋
    delay(1000);

    Serial1.begin(115200); //통신속도 115200
    delay(1000);
    //Serial1.write("AT+POWE3"); //출력파워 6db으로 조정, 가장 강하게 씀
    //delay(1000);
//    randomSeed(analogRead(5));
//    int a,b,c,d;
//    a = random(0x30,0x39);
//    b = random(0x30,0x39);
//    c = random(0x30,0x39);
//    d = random(0x30,0x39);
    Serial1.write("AT+NAME"); //이름을 AIR0000~AIR9999 까지 랜덤으로 설정
    Serial1.print(bleName);
    delay(1000);
//    Serial1.write(a);
//    Serial1.write(b);
//    Serial1.write(c);
//    Serial1.write(d);
//    delay(1000);
    
    Serial1.write("AT+RESET"); //리셋
    delay(1000);
}

void MasterSet(){
    Serial1.begin(115200); //통신속도 115200
    delay(1000);
    Serial1.write("AT+RENEW"); //설정 초기화
    delay(2000);
    Serial1.write("AT+RESET"); //리셋
    delay(1000);

    Serial1.begin(9600); //통신속도 9600
    delay(1000);
    Serial1.write("AT+RENEW"); //설정 초기화
    delay(2000);
    Serial1.write("AT+RESET"); //리셋
    delay(1000);

    Serial1.write("AT"); //AT
    delay(1000);
    Serial1.write("AT+BAUD4"); //통신속도 115200으로 변경
    delay(1000);
    Serial1.write("AT+RESET"); //리셋
    delay(1000);

    Serial1.begin(115200); //통신속도 115200
    delay(1000);
    //Serial1.write("AT+POWE3"); //출력파워 6db으로 조정, 가장 강하게 씀
    //delay(1000);
    Serial1.write("AT+ROLE1"); //MASTER로 설정
    delay(1000);
    Serial1.write("AT+RESET"); //리셋
    delay(1000);
}
