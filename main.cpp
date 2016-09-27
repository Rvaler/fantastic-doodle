#include "library/environm.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI  3.1415

float fuzzificador(float pontoA, float pontoB, float pontoC, float entrada)
{
    //Se a entrada estiver fora do triângulo, retorna zero
    if (entrada < pontoA || entrada > pontoC) return 0.0;

    //senão verifica se está antes do pontoB
    if (entrada <= pontoB)
    {
        return (entrada - pontoA) / (pontoB - pontoA);
    }
    else
    {
        //ou depois do pontoB
        return (pontoC - entrada) / (pontoC - pontoB);
    }
}

//retorna o valor máximo entre 2 valores
float retornaMaximo(float valorA, float valorB)
{
	if (valorA > valorB) {
		return valorA;
	}
	else {
		return valorB;
	}
}

//retorna o valor mínimo entre 2 valores
float retornaMinimo(float valorA, float valorB)
{
	if (valorA < valorB) {
		return valorA;
	}
	else {
		return valorB;
	}
}

int main( int argc, char* argv[] ) {

    // Declaração do objeto que representa o ambiente.
    environm::soccer::clientEnvironm environment;

    // *************************************** CONEXÃO ***************************************
//    printf("Enter Server Adress: ");
//    scanf("%s",&serverAdress);
//    printf("Port Number:");
//    scanf("%d",&serverPort);

    if ( argc != 3 ) {
        printf("\nInvalid parameters. Expecting:");
        printf("\nSoccerPlayer SERVER_ADDRESS_STRING SERVER_PORT_NUMBER\n");
        return 0;
    }
	//cout << "Server: " << argv[1] << " - Port: " << atoi(argv[2]) << endl;
    // Conecta-se ao SoccerMatch. SupÃµe que SoccerMatch estÃ¡ rodando na mÃ¡quina
    // local e que um dos robÃ´s esteja na porta 1024. Porta e local podem mudar.
    if ( ! environment.connect( argv[1], atoi( argv[2] ) ) ) {
       printf("\nFail connecting to the SoccerMatch.\n");
        return 0;  // Cancela operação se não conseguiu conectar-se.
    }

    //Conjuntos utilizados e valores:

    //..................................ESQUERDA..,.FRENTE.,..DIREITA....
    float anguloConjuntoBola[3][3] = {{-PI,-PI/2,0},{-PI/2,0,PI/2},{0,PI/2,PI}};
    float anguloConjuntoAlvo[3][3] = {{-PI,-PI/2,0},{-PI/2,0,PI/2},{0,PI/2,PI}};
    float anguloConjuntoObstaculo[3][3] = {{-PI,-PI/2,0},{-PI/2,0,PI/2},{0,PI/2,PI}};
    float rotacaoConjuntoRobo[3][3] = {{-PI,-PI/2,0},{-PI/2,0,PI/2},{0,PI/2,PI}};

    //.........................................PERTO..,....MÉDIO...,.....LONGE......
    float conjuntoDistanciaObstaculo[3][3] = {{0,1,50},{50,500,950},{500,1985,1990}};

    printf( "\nRunning..." );

    while (1)
    {
        //Pega os valores do Servidor(Ambiente)
        float anguloBolaAmbiente = environment.getBallAngle();
        float anguloAlvoAmbiente = environment.getTargetAngle( environment.getOwnGoal() );
        float distanciaObstaculo = environment.getCollision();
        float anguloObstaculo = environment.getObstacleAngle();

        //Processo de Fuzzificação------------------------------------------------------------
        //Desativa todas regras que podem ser ativadas
        bool ativaRegraBolaE = false;
        bool ativaRegraBolaF = false;
        bool ativaRegraBolaD = false;
        bool ativaRegraAlvoE = false;
        bool ativaRegraAlvoF = false;
        bool ativaRegraAlvoD = false;
        bool ativaRegraDistanciaE = false;
        bool ativaRegraDistanciaF = false;
        bool ativaRegraDistanciaD = false;
        bool ativaRegraObstaculoE = false;
        bool ativaRegraObstaculoF = false;
        bool ativaRegraObstaculoD = false;

        //Zera todos valores para que novos sejam calculados
        float valorFuzzificadorBolaE = 0;
        float valorFuzzificadorBolaF = 0;
        float valorFuzzificadorBolaD = 0;
        float valorFuzzificadorAlvoE = 0;
        float valorFuzzificadorAlvoF = 0;
        float valorFuzzificadorAlvoD = 0;
        float valorFuzzificadorDistanciaPequena = 0;
        float valorFuzzificadorDistanciaMedia = 0;
        float valorFuzzificadorDistanciaLonga = 0;
        float valorFuzzificadorObstaculoPequena = 0;
        float valorFuzzificadorObstaculoMedia = 0;
        float valorFuzzificadorObstaculoLonga = 0;

        //Zera os valores de cada Regra de Esquerda, Frente e Direita
        float valorTotalRegrasE = 0;
        float valorTotalRegrasF = 0;
        float valorTotalRegrasD = 0;


        // Verifica ativação de ANGULO BOLA

        if (anguloConjuntoBola[0][0] <= anguloBolaAmbiente && anguloBolaAmbiente <= anguloConjuntoBola[0][2]) { //verifica se o valor está entre -PI e 0, se estiver pertence ao conjunto E
            ativaRegraBolaE = true;
            valorFuzzificadorBolaE = fuzzificador(anguloConjuntoBola[0][0],anguloConjuntoBola[0][1],anguloConjuntoBola[0][2],anguloBolaAmbiente);
        }
        if (anguloConjuntoBola[1][0] <= anguloBolaAmbiente && anguloBolaAmbiente <= anguloConjuntoBola[1][2]) { //verifica se o valor está entre -1 e 1, se estiver pertence ao conjunto F
            ativaRegraBolaF = true;
            valorFuzzificadorBolaF = fuzzificador(anguloConjuntoBola[1][0],anguloConjuntoBola[1][1],anguloConjuntoBola[1][2],anguloBolaAmbiente);
        }
        if (anguloConjuntoBola[2][0] <= anguloBolaAmbiente && anguloBolaAmbiente <= anguloConjuntoBola[2][2]) { //verifica se o valor está entre 0 e PI, se estiver pertence ao conjunto D
            ativaRegraBolaD = true;
            valorFuzzificadorBolaD = fuzzificador(anguloConjuntoBola[2][0],anguloConjuntoBola[2][1],anguloConjuntoBola[2][2],anguloBolaAmbiente);
        }

        // Verifica ativação de ANGULO ALVO

        if (anguloConjuntoAlvo[0][0] <= anguloAlvoAmbiente && anguloAlvoAmbiente <= anguloConjuntoAlvo[0][2]) { //verifica se o valor está entre -PI e 0, se estiver pertence ao conjunto E
            ativaRegraAlvoE = true;
            valorFuzzificadorAlvoE = fuzzificador(anguloConjuntoAlvo[0][0],anguloConjuntoAlvo[0][1],anguloConjuntoAlvo[0][2],anguloAlvoAmbiente);
        }
        if (anguloConjuntoAlvo[1][0] <= anguloAlvoAmbiente && anguloAlvoAmbiente <= anguloConjuntoAlvo[1][2]) { //verifica se o valor está entre -1 e 1, se estiver pertence ao conjunto F
            ativaRegraAlvoF = true;
            valorFuzzificadorAlvoF = fuzzificador(anguloConjuntoAlvo[1][0],anguloConjuntoAlvo[1][1],anguloConjuntoAlvo[1][2],anguloAlvoAmbiente);
        }
        if (anguloConjuntoAlvo[2][0] <= anguloAlvoAmbiente && anguloAlvoAmbiente <= anguloConjuntoAlvo[2][2]) { //verifica se o valor está entre 0 e PI, se estiver pertence ao conjunto D
            ativaRegraAlvoD = true;
            valorFuzzificadorAlvoD = fuzzificador(anguloConjuntoAlvo[2][0],anguloConjuntoAlvo[2][1],anguloConjuntoAlvo[2][2],anguloAlvoAmbiente);
        }

        // DISTANCIA ATE OBSTACULO

        if (conjuntoDistanciaObstaculo[0][0] <= distanciaObstaculo && distanciaObstaculo <= conjuntoDistanciaObstaculo[0][2]) {
             // Ativa se o obstáculo estiver a uma distância pequena
            ativaRegraDistanciaE = true;
            valorFuzzificadorDistanciaPequena = fuzzificador(conjuntoDistanciaObstaculo[0][0],conjuntoDistanciaObstaculo[0][1],conjuntoDistanciaObstaculo[0][2],distanciaObstaculo);
        }
        if (conjuntoDistanciaObstaculo[1][0] <= distanciaObstaculo && distanciaObstaculo <= conjuntoDistanciaObstaculo[1][2]) {
            // Ativa se o obstáculo estiver a uma distância média
            ativaRegraDistanciaF = true;
            valorFuzzificadorDistanciaMedia = fuzzificador(conjuntoDistanciaObstaculo[1][0],conjuntoDistanciaObstaculo[1][1],conjuntoDistanciaObstaculo[1][2],distanciaObstaculo);
        }
        if (conjuntoDistanciaObstaculo[2][0] <= distanciaObstaculo && distanciaObstaculo <= conjuntoDistanciaObstaculo[2][2]) {
             // Ativa se o obstáculo estiver a uma distância longa
            ativaRegraDistanciaD = true;
            valorFuzzificadorDistanciaLonga = fuzzificador(conjuntoDistanciaObstaculo[2][0],conjuntoDistanciaObstaculo[2][1],conjuntoDistanciaObstaculo[2][2],distanciaObstaculo);
        }

        // ANGULO OBSTACULO

        if (anguloConjuntoObstaculo[0][0] <= anguloObstaculo && anguloObstaculo <= anguloConjuntoObstaculo[0][2]) {
            // ativa se o obstaculo estiver a esquerda
            ativaRegraObstaculoE = true;
            valorFuzzificadorObstaculoPequena = fuzzificador(anguloConjuntoObstaculo[0][0],anguloConjuntoObstaculo[0][1],anguloConjuntoObstaculo[0][2],anguloObstaculo);
        }
        if (anguloConjuntoObstaculo[1][0] <= anguloObstaculo && anguloObstaculo <= anguloConjuntoObstaculo[1][2]) {
            // ativa se o obstaculo estiver a frente
            ativaRegraObstaculoF = true;
            valorFuzzificadorObstaculoMedia = fuzzificador(anguloConjuntoObstaculo[1][0],anguloConjuntoObstaculo[1][1],anguloConjuntoObstaculo[1][2],anguloObstaculo);
        }
        if (anguloConjuntoObstaculo[2][0] <= anguloObstaculo && anguloObstaculo <= anguloConjuntoObstaculo[2][2]) {
            // ativa se o obstaculo estiver a direita
            ativaRegraObstaculoD = true;
            valorFuzzificadorObstaculoLonga = fuzzificador(anguloConjuntoObstaculo[2][0],anguloConjuntoObstaculo[2][1],anguloConjuntoObstaculo[2][2],anguloObstaculo);
        }

        //Calcula valores das regras, aqui vai o cálculo do Mamdami------------------------------------------------------------

        if (ativaRegraBolaE && ativaRegraAlvoE) { //regra R1 ativa
            valorTotalRegrasF = retornaMaximo(retornaMinimo(valorFuzzificadorBolaE,valorFuzzificadorAlvoE), valorTotalRegrasF);
        }
        if (ativaRegraBolaE && ativaRegraAlvoF) { //regra R2 ativa
            valorTotalRegrasE = retornaMaximo(retornaMinimo(valorFuzzificadorBolaE,valorFuzzificadorAlvoF), valorTotalRegrasE);
        }
        if (ativaRegraBolaE && ativaRegraAlvoD) { //regra R3 ativa
            valorTotalRegrasE = retornaMaximo(retornaMinimo(valorFuzzificadorBolaE,valorFuzzificadorAlvoD), valorTotalRegrasE);
        }
        if (ativaRegraBolaF && ativaRegraAlvoE) { //regra R4 ativa
            valorTotalRegrasD = retornaMaximo(retornaMinimo(valorFuzzificadorBolaF,valorFuzzificadorAlvoE), valorTotalRegrasD);
        }
        if (ativaRegraBolaF && ativaRegraAlvoF) { //regra R5 ativa
            valorTotalRegrasF = retornaMaximo(retornaMinimo(valorFuzzificadorBolaF,valorFuzzificadorAlvoF), valorTotalRegrasF);
        }
        if (ativaRegraBolaF && ativaRegraAlvoD) { //regra R6 ativa
            valorTotalRegrasE = retornaMaximo(retornaMinimo(valorFuzzificadorBolaF,valorFuzzificadorAlvoD), valorTotalRegrasE);
        }
        if (ativaRegraBolaD && ativaRegraAlvoE) { //regra R7 ativa
            valorTotalRegrasD = retornaMaximo(retornaMinimo(valorFuzzificadorBolaD,valorFuzzificadorAlvoE), valorTotalRegrasD);
        }
        if (ativaRegraBolaD && ativaRegraAlvoF) { //regra R8 ativa
            valorTotalRegrasD = retornaMaximo(retornaMinimo(valorFuzzificadorBolaD,valorFuzzificadorAlvoF), valorTotalRegrasD);
        }
        if (ativaRegraBolaD && ativaRegraAlvoD) { //regra R9 ativa
            valorTotalRegrasF = retornaMaximo(retornaMinimo(valorFuzzificadorBolaD,valorFuzzificadorAlvoD), valorTotalRegrasF);
        }

        // Regras de Obstáculo
        // perto de obstáculo e alvo (gol) a esquerda
        if (ativaRegraDistanciaE && ativaRegraAlvoE) {
            valorTotalRegrasE = retornaMaximo(retornaMinimo(valorFuzzificadorDistanciaPequena,valorFuzzificadorAlvoE), valorTotalRegrasE);
        }
        // perto de obstáculo e obstáculo a esquerda -> vira pra direita
        if (ativaRegraDistanciaE && ativaRegraObstaculoE) {
            valorTotalRegrasD = retornaMaximo(retornaMinimo(ativaRegraDistanciaE,valorFuzzificadorObstaculoPequena), valorTotalRegrasD);
        }
        // perto de obstáculo e obstáculo a direita -> vira pra esquerda
        if (ativaRegraDistanciaE && ativaRegraObstaculoD) {
            valorTotalRegrasE = retornaMaximo(retornaMinimo(ativaRegraDistanciaE,valorFuzzificadorObstaculoLonga), valorTotalRegrasE);
        }
        // perto de obstáculo e alvo (gol) a frente
        if (ativaRegraDistanciaE && ativaRegraAlvoF) {
            valorTotalRegrasF = retornaMaximo(retornaMinimo(valorFuzzificadorDistanciaPequena,valorFuzzificadorAlvoF), valorTotalRegrasF);
        }
        // média dist de obstáculo e alvo (gol) a frente
        if (ativaRegraDistanciaF && ativaRegraAlvoF) {
            valorTotalRegrasF = retornaMaximo(retornaMinimo(valorFuzzificadorDistanciaMedia,valorFuzzificadorAlvoF), valorTotalRegrasF);
        }

        //Processo de Defuzificação ------------------------------------------------------------------------------------------------------
        float sumY = 0;
        float sumXY = 0;

        for(float x = -PI; x <= PI; x+= 0.1)
        {
            float y = 0;
            if(rotacaoConjuntoRobo[0][0] <= x && x <= rotacaoConjuntoRobo[0][2])
            {
                y = retornaMaximo(retornaMinimo(fuzzificador(rotacaoConjuntoRobo[0][0],rotacaoConjuntoRobo[0][1],rotacaoConjuntoRobo[0][2], x), valorTotalRegrasE), y);
            }

            if(rotacaoConjuntoRobo[1][0] <= x && x <= rotacaoConjuntoRobo[1][2])
            {
                y = retornaMaximo(retornaMinimo(fuzzificador(rotacaoConjuntoRobo[1][0],rotacaoConjuntoRobo[1][1],rotacaoConjuntoRobo[1][2], x), valorTotalRegrasF), y);
            }

            if(rotacaoConjuntoRobo[2][0] <= x && x <= rotacaoConjuntoRobo[2][2])
            {
                y = retornaMaximo(retornaMinimo(fuzzificador(rotacaoConjuntoRobo[2][0],rotacaoConjuntoRobo[2][1],rotacaoConjuntoRobo[2][2], x), valorTotalRegrasD), y);
            }
            sumXY += x * y;
            sumY += y;
        }

        float motor = sumXY / sumY;
        motor /= 1.5;



        float leftMotor  = (cos(motor) - sin(motor)) / 2.5;
        float rightMotor = (cos(motor) + sin(motor)) / 2.5;

        // Transmite ação do robô ao ambiente. Fica bloqueado até que todos os
        // robôs joguem. Se erro, retorna false (neste exemplo, sai do laco).
        if ( ! environment.act( leftMotor, rightMotor ) ) {
            break; // Termina a execução se falha ao agir.
        }
    }

    return 0;
}
//------------------------------------------------------------------------------

