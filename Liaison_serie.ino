#include <AccelStepper.h>
#include <ctype.h>

// Pin pour les impulsions du moteur
const int pinPULSx = 9;
// Pin pour la direction du moteur
const int pinDIRx = 10;

// Pin pour les impulsions du moteur
const int pinPULSy = 7;
// Pin pour la direction du moteur
const int pinDIRy = 6;

unsigned long startTime;
unsigned long endTime;
unsigned long executionTime;

char Xi[4];
char X_prec[4];
char Yi[4];

char coord;
bool flag_error_x;
bool flag_error_y;
bool lancer_en_cours_x = false;
bool lancer_en_cours_y = false;

int x_goal = 0;
int y_goal = 0;
int x_goal_prec = 0;
int pas_a_faire_x = 0;
int pas_a_faire_y = 0;
int pas_par_tr = 3200;

// Création de l'objet AccelStepper pour contrôler le moteur
AccelStepper stepper_x(AccelStepper::DRIVER, pinPULSx, pinDIRx);
AccelStepper stepper_y(AccelStepper::DRIVER, pinPULSy, pinDIRy);

void deplac_y(){
  if (y_goal == -1740){
    stepper_y.moveTo(0);
    stepper_y.setMaxSpeed(4500);
  }
  else if (y_goal > -300 && y_goal < 300){
    stepper_y.setMaxSpeed(4500);
    stepper_y.moveTo(pas_a_faire_y);
    lancer_en_cours_y = true;
  } 
}

void setup() {
  Serial.begin(38400);
  // delay(2000);

  while (!Serial) {};   // Attendre que le port série soit prêt

//Moteur x
  // Définit la vitesse maximale du moteur
  stepper_x.setMaxSpeed(2500);
  // Définit la valeur d'accélération
  stepper_x.setAcceleration(1730000);
  stepper_x.setCurrentPosition(0);

//Moteur y
  // Définit la vitesse maximale du moteur
  stepper_y.setMaxSpeed(300);
  // Définit la valeur d'accélération
  stepper_y.setAcceleration(800000);
  stepper_y.setCurrentPosition(0);

}

void loop() {
  // startTime = micros();
  while(Serial.available() > 5){
    coord = Serial.read();
    flag_error_x = false;
    flag_error_y = false;
    if(coord == 'x'){
      for (int i = 0; i < 4; i++){
        Xi[i] = Serial.read();
        if (!isdigit(Xi[i]) && Xi[i] != '-'){flag_error_x = true;}
      }
    if (!flag_error_x){x_goal = atoi(Xi);}
    Serial.print("X = ");
    Serial.print(x_goal);
    pas_a_faire_x = ceil((x_goal/65.0)*pas_par_tr);
    }
    else if(coord == 'y'){
      for (int i = 0; i < 4; i++){
        Yi[i] = Serial.read();
        if (!isdigit(Yi[i])){flag_error_y = true;}
      }
      if (!flag_error_y){y_goal = atoi(Yi) - 1740;}
      Serial.print("Y = ");
      Serial.print(y_goal);
      pas_a_faire_y = ceil((y_goal/65.0)*pas_par_tr);
    }
  }

  if (x_goal == 0){
    stepper_x.moveTo(0);
    deplac_y();
    while(stepper_x.distanceToGo() != 0 || stepper_y.distanceToGo() != 0){
      stepper_x.run();
      stepper_y.run();
    } 
    if (lancer_en_cours_x && lancer_en_cours_y){
      while (Serial.available() != 0){
        Serial.read();
      }
      lancer_en_cours_x = false;
      lancer_en_cours_y = false;
    }
  }
  else if (x_goal > -300 && x_goal < 300){
    stepper_x.moveTo(pas_a_faire_x);
    lancer_en_cours_x = true;
    deplac_y();
    while(stepper_x.distanceToGo() != 0 || stepper_y.distanceToGo() != 0){
      stepper_x.run();
      stepper_y.run();
    } 
  } 

  // endTime = micros();
  // executionTime = endTime - startTime;
  // Serial.print(executionTime);
  
  // stepper.stop();
}

