
/* Declaração das variáveis para acionamento dos motores*/
const int OUTA = 3;
const int OUTB = 9;
const int OUTC = 10;
const int OUTD = 11;

/* Declarando os pino do arduino de acionamento dos motores como saída*/
void init_motores(){
  pinMode(OUTA, OUTPUT);
  pinMode(OUTB, OUTPUT);
  pinMode(OUTC, OUTPUT);
  pinMode(OUTD, OUTPUT);
  
}

void PMWControleMotores(double comando){

  if(comando > 0){
    analogWrite(OUTA, 0);                         /* Controlando o motor da direita para trás*/
    analogWrite(OUTB, abs(comando));              /* Controlando o motor da direita para frente*/
    analogWrite(OUTC, 0);                         /* Controlando o motor da esquerda para trás*/
    analogWrite(OUTD, abs(comando));              /* Controlando o motor da esquerda para frente*/
  }else{
    analogWrite(OUTA, abs(comando));              /* Controlando o motor da direita para trás*/
    analogWrite(OUTB, 0);                         /* Controlando o motor da direita para frente*/
    analogWrite(OUTC, abs(comando));              /* Controlando o motor da esquerda para trás*/
    analogWrite(OUTD, 0);                         /* Controlando o motor da esquerda para frente*/
    
  }
}

