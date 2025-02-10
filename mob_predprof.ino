#define l_dir 4
#define L_speed 5
#define R_speed 6
#define r_dir 7
#define L_line A0
#define R_line A1
#define GRAY_L 533
#define GRAY_R 552
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
#define S0 11
#define S1 10
#define S2 12
#define S3 13
#define sensorOut 8

#define left_phase_a 2
#define left_phase_b A2

#define right_phase_a 3
#define right_phase_b A3

volatile long left_curr_pos;
int left_targ_pos;

void left_enc_phase_a() {
  if (digitalRead(left_phase_b)) {
    left_curr_pos++;
  } else {
    left_curr_pos--;
  }
}

volatile long right_curr_pos;
int right_targ_pos;

void right_enc_phase_a() {
  if (digitalRead(right_phase_b)) {
    right_curr_pos--;
  } else {
    right_curr_pos++;
  }
}

void reset_enc() {
  left_curr_pos = 0;
  right_curr_pos = 0;
  right_targ_pos = 0;
  left_targ_pos = 0;
}

void go_n_tick(long n) {
  reset_enc();
  int left_go = 100;
  int right_go = 100;
  while (true) {
    go(left_go, right_go);
    if (left_curr_pos >= n and right_curr_pos >= n) {
      stopp();
      return;
    } else if (right_curr_pos >= n) {
      right_go = 0;
    } else if (left_curr_pos >= n) {
      left_go = 0;
    }
  }
}

const int mm_tk = 610;
void go_n_mm(long n) {
  reset_enc();
  int mm = 1000 * n / mm_tk;
  go_n_tick(mm);
}

void turn_l_n_tick(long n) {
  reset_enc();
  int left_go = -100;
  int right_go = 100;
  while (true) {
    go(left_go, right_go);
    if (left_curr_pos <= n and right_curr_pos >= n) {
      stopp();
      return;
    } else if (right_curr_pos >= n) {
      right_go = 0;
    } else if (left_curr_pos <= n - 2 * n) {
      left_go = 0;
    }
  }
}

const int grad_l_tk = 157;
void turn_l_n_grad(long n) {
  reset_enc();
  int mm = 500 * n / grad_l_tk;
  turn_l_n_tick(mm);
}

void turn_r_n_tick(long n) {
  reset_enc();
  int left_go = 100;
  int right_go = -100;
  while (true) {
    go(left_go, right_go);
    if (left_curr_pos >= n and right_curr_pos <= n) {
      stopp();
      return;
    } else if (right_curr_pos <= n - 2 * n) {
      right_go = 0;
    } else if (left_curr_pos >= n) {
      left_go = 0;
    }
  }
}

const int grad_r_tk = 157;
void turn_r_n_grad(long n) {
  reset_enc();
  int mm = 500 * n / grad_r_tk;
  turn_r_n_tick(mm);
}

void sinhron(long n, int speed, float k = 0.2) {
  reset_enc();
  while (true) {
    int err = left_curr_pos - right_curr_pos;
    int u = err * k;
    go(speed - u, speed + u);
    if (left_curr_pos >= n and right_curr_pos >= n) {
      stopp();
      return;
    }
  }
}

void sinhron_mm(long mm, int speed, float k = 0.2) {
  reset_enc();
  int n = 1000 * mm / mm_tk;
  while (true) {
    int err = left_curr_pos - right_curr_pos;
    int u = err * k;
    go((speed - u) * 0.9, speed + u);
    if (left_curr_pos >= n and right_curr_pos >= n) {
      stopp();
      return;
    }
  }
}

int frequency = 0;
int color = 0;

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Stores the red. green and blue colors
int redColor = 0;
int greenColor = 0;
int blueColor = 0;

void setup() {
  pinMode(l_dir, OUTPUT);
  pinMode(L_speed, OUTPUT);
  pinMode(r_dir, OUTPUT);
  pinMode(R_speed, OUTPUT);
  pinMode(L_line, INPUT);
  pinMode(R_line, INPUT);

  pinMode(left_phase_a, INPUT);
  pinMode(left_phase_b, INPUT);
  attachInterrupt(
    digitalPinToInterrupt(left_phase_a),
    left_enc_phase_a, RISING);

  pinMode(right_phase_a, INPUT);
  pinMode(right_phase_b, INPUT);
  attachInterrupt(
    digitalPinToInterrupt(right_phase_a),
    right_enc_phase_a, RISING);

  // Setting the outputs
  //pinMode(S0, OUTPUT);
  //pinMode(S1, OUTPUT);
  //pinMode(S2, OUTPUT);
  //pinMode(S3, OUTPUT);

  // Setting the sensorOut as an input
  //pinMode(sensorOut, INPUT);

  // Setting frequency scaling to 20%
  //digitalWrite(S0, HIGH);
  //digitalWrite(S1, LOW);

  attachInterrupt(
    digitalPinToInterrupt(left_phase_a),
    left_enc_phase_a,
    RISING
  );
  Serial.begin(9600);
  myservo.attach(9);
  myservo.write(50);
  delay(100);


}

//задаем направление и скрость моторам
void go(long l_speed, long r_speed) {
  l_speed = constrain(l_speed, -255, 255);
  digitalWrite(l_dir, !(l_speed <= 0));
  analogWrite(L_speed, abs(l_speed));

  r_speed = constrain(r_speed, -255, 255);
  digitalWrite(r_dir, !(r_speed <= 0));
  analogWrite(R_speed, abs(r_speed));
}

//обработка правого датчика линии
int r_line() {
  return analogRead(R_line);
}

//обработка левого датчика линии
int l_line() {
  return analogRead(L_line);
}

#define PK 0.3
#define PD 1.8

int last_err = 0;

//пропорция
void pd(int Speed) {
  short err = l_line() - r_line();
  short u = err * PK + (-last_err + err) * PD;
  last_err = err;
  go(Speed + u, Speed - u);
  delay(5);
}

//проверка, что левый датчик на черной линии
bool l_on_line() {
  return l_line() < GRAY_L;
}

//проверка, что правый датчик на черной линии
bool r_on_line() {
  return r_line() < GRAY_R;
}

//функция торможения
void stopp() {
  for (short i = 0; i < 5; i++) {
    go(120, 120);
    delay(10);
    go(-120, -120);
    delay(10);
  }
  go(0, 0);
}

//езда до Н-ого перекрестка
void go_to(int n, const unsigned short speed = 100) {
  int count = 0;
  bool was_on_cross = false;

  while (count < n) {
    pd(speed);

    if (l_on_line() and r_on_line()) {
      if (not was_on_cross) {
        was_on_cross = true;
        go(100, 100);
        delay(50);
        count++;
      }
    } else {
      was_on_cross = false;
    }
  }
  sinhron_mm(100, 100);
  stopp();
}

//поворот направо с ориентировкой по линии
void turn_r_n(int n = 1) {
  int count = 0;
  bool was_on_cross = false;
  //  while(not l_on_line()){
  //go(100, -100);
  //}
  go(100, -100);
  delay(200);
  stopp();
  while (true) {
    go(100, -100);

    if (r_on_line()) {
      if (not was_on_cross) {
        was_on_cross = true;
        count++;
      }
    } else {
      was_on_cross = false;
    }
    if (count == n) {
      while (not l_on_line()) {
        go(100, -100);
      }
      go(-100, 100);
      delay(50);
      stopp();
      return;
    }
  }
}

//поворот налево с ориентировкой по линии
void turn_l_n(int n = 1) {
  int count = 0;
  bool was_on_cross = false;
  //while(not r_on_line()){
  //go(-100, 100);
  //}
  go(-100, 100);
  delay(200);
  stopp();
  while (true) {
    go(-100, 100);

    if (l_on_line()) {
      if (not was_on_cross) {
        was_on_cross = true;
        count++;
      }
    } else {
      was_on_cross = false;
    }
    if (count == n) {
      while (not r_on_line()) {
        go(-100, 100);
      }
      go(100, -100);
      delay(50);
      stopp();
      return;
    }
  }
}

void l_time(unsigned long n) {
  unsigned long t = millis();
  while (millis() < t + n) {
    if (l_on_line()) {
      go(0, 100);
    } else {
      go(100, 0);
    }
  }
}


void r_time(unsigned long n) {
  unsigned long t = millis();
  while (millis() < t + n) {
    if (r_on_line()) {
      go(100, 0);
    } else {
      go(0, 100);
    }
  }
}



// функция распознавания цвета
void readColor() {
  // устанавливаем фильтр для красного
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  //считываем значение на выходе
  frequency = pulseIn(sensorOut, LOW);
  //значение красного
  int R = frequency;
  Serial.print("R= ");
  Serial.print(frequency);
  Serial.print("  ");
  delay(50);

  //устанавливаем фильтр для зеленого
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  //считываем значение на выходе
  frequency = pulseIn(sensorOut, LOW);
  int G = frequency;
  //значение зеленого
  Serial.print("G= ");
  Serial.print(frequency);
  Serial.print("  ");
  delay(50);

  // устанавливаем фильтр для синего
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  //считываем значение на выходе
  frequency = pulseIn(sensorOut, LOW);
  int B = frequency;
  //значение синего
  Serial.print("B= ");
  Serial.print(frequency);
  Serial.println("  ");
  delay(50);

  //все, что написано выше необходимо для калибровки. Для каждого кубика необходимо измерить предельные значения 2-х цветов, указаных ниже для каждого

  if (R<200 & R>50 & G <= 180 & G > 42 & B<285 & B>62 & R > G) {
    color = 0; // Green
    Serial.println("Green");
  }
  else if (R<130 & R>30 & G<480 & G>115 & B<380 & B>110 & R < G) {
    color = 1; // Red
    Serial.println("Red");
  }
  else if (G<58 & G>45 & B<40 & B>26) {
    color = 2  ; // Blue
    Serial.println("Blue");
  } else {
    Serial.println("None");
    color = 3;
  }
  return color;
}

int start[4];
int over[4];

void code_line() {
  int i = 0;
  bool flag = true;
  while (i != 8) {
    readColor();
    if (flag and color != 3) {
      i++;
      if (i <= 4) {
        start[i] += color;
      } else {
        over[i - 4] += color;
      }
      flag = false;
    }
    if (color == 3) {
      flag = true;
    }

    pd(150);
  }
  stopp();
}

//езда до Н-ого перекрестка без выравнивания
void go_to_line(int n, int speed = 100, float k = 0.2) {
  int count = 0;
  bool was_on_cross = false;
  reset_enc();
  while (count < n) {
    int err = left_curr_pos - right_curr_pos;
    int u = err * k;
    go((speed - u) * 0.9, speed + u);

    if (l_on_line() and r_on_line()) {
      if (not was_on_cross) {
        was_on_cross = true;
        go(100, 100);
        delay(50);
        count++;
      }
    } else {
      was_on_cross = false;
    }
  }
}


void zigzag_to_n_cross(int n) {
  go_to(n);
}

void turn_right() {
  turn_r_n(1);
}

void turn_left() {
  turn_l_n(1);
}

void turn_back() {
  go(-100, -100);
  delay(200);
}
































void go_line_n_mm(long n, int speed) {
  long mm = 1000 * n / mm_tk;
  reset_enc();
  while (true) {
    pd(speed);
    if (left_curr_pos >= mm or right_curr_pos >= mm) {
      stopp();
      return;
    }
  }
}




void go_to_obj( const unsigned short speed = 100) {
  int count = 0;
  bool was_on_cross = false;

  while (true) {
    pd(speed);

    if ((61.681 * pow(analogRead(A4) * 0.0048828125, -1.133)) < 20.0) {
      stopp();
      return;
    }
  }
}




void loop() {
  go_to(1);
  turn_l_n();
  go_to(1);
  turn_l_n();
  go_to(1);
  turn_r_n();
  go_to_obj();
  go(-100, -100);
  delay(600);
  turn_r_n_grad(85);
  myservo.write(180);
  delay(100);
  sinhron_mm(200, 100);
  turn_l_n_grad(80);
  sinhron_mm(450, 100);
  turn_l_n_grad(75);
  go_to(1);
  myservo.write(50);
  delay(100);
  turn_r_n();
  go_to(1);
  turn_r_n();
  go_to(1);
  turn_r_n();
  go_line_n_mm(300, 100);
  turn_l_n();
  go_to(1);
  turn_r_n();
  go_to(1);
  turn_r_n();


  go_to_obj();
  myservo.write(180);
  delay(100);

  turn_l_n_grad(70);
  go(100, 100);
  delay(300);
  stopp();
  myservo.write(50);
  delay(100);
  go(-100, -100);
  delay(300);
  turn_r_n();
  go_to(1);
  turn_l_n();

  go_to(1);
  turn_r_n();
  go_to(1);
  turn_l_n();


  go_to_obj();
  go(-100, -100);
  delay(600);
  turn_l_n_grad(70);
  myservo.write(180);
  delay(100);
  sinhron_mm(150, 100);
  turn_r_n_grad(80);
  sinhron_mm(510, 100);
  turn_r_n_grad(70);
  go_to(1);
  myservo.write(50);
  delay(100);
  turn_l_n();
  go_to(1);
  turn_l_n();
  go_to_obj();

  myservo.write(180);
  delay(1000);



  go(-100, -100);
  delay(900);
  turn_l_n();
  go(100, 100);
  delay(200);
  stopp();
  go(100, -100);
  delay(100);
  go_line_n_mm(200, 100);
  go_line_n_mm(200, 100);
  turn_r_n_grad(85);
  sinhron_mm(150, 100);
  turn_l_n_grad(70);
  sinhron_mm(550, 100);
  turn_l_n_grad(70);
  go_to(1);
  turn_r_n();
  go_to(1);
  turn_l_n();
  go_to(1);
  turn_r_n();
  go_to(1);
  turn_l_n();
  go_to(1);
  turn_r_n();
  go_to(1);
  turn_l_n();
  go_line_n_mm(50, 100);
  turn_l_n_grad(70);
  sinhron_mm(200, 100);
  turn_r_n_grad(80);
  sinhron_mm(550, 100);
  turn_r_n_grad(70);
  go_to(1);
  turn_l_n();
  go_to(1);
  turn_r_n();
  go_to(1);
  turn_r_n();
  go_to(1);
  go(-100, -100);
  delay(200);
  stopp();
  myservo.write(50);
  delay(1000);
  

  //go_to(1);
  //turn_r_n();






  stopp();
  exit(0);
}
