#include "library/environm.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI  3.14159265358979323846

float fuzzify(float point_A, float point_B, float point_C, float input)
{
    //Se a entrada estiver fora do triângulo, retorna zero
    if (input < point_A || input > point_C) { return 0; }
    //senão verifica se está antes do pontoB
    if (input <= point_B) { return (input - point_A) / (point_B - point_A); }
    //ou depois do pontoB
    return (point_C - input) / (point_C - point_B);
}


float max(float a, float b)
{
	if (a > b) { return a; }
	return b;
}
float min(float a, float b)
{
	if (a < b) { return a; }
	return b;
}

int main( int argc, char* argv[] ) {

    // Representação do ambiente
    environm::soccer::clientEnvironm environment;

    // Conexão
    if ( argc != 3 ) {
        printf("\nInvalid parameters. Expecting:");
        printf("\nSoccerPlayer SERVER_ADDRESS_STRING SERVER_PORT_NUMBER\n");
        return 0;
    }
    if ( ! environment.connect( argv[1], atoi( argv[2] ) ) ) {
       printf("\nFail connecting to the SoccerMatch.\n");
        return 0;  // Cancela operação se não conseguiu conectar-se.
    }

    //Conjuntos utilizados e valores:

    //                         Esquerda     Frente       Direita
    float angles[3][3] = {{-PI,-PI/2,0},{-PI/2,0,PI/2},{0,PI/2,PI}};

    //                         Próximo     Médio       Distante
    float distances[3][3] = {{0,1,50},{50,500,950},{500,1985,1990}};

    while (true)
    {
        // Valores providos do ambiente
        float angle_ball = environment.getBallAngle();
        float angle_target = environment.getTargetAngle( environment.getOwnGoal() );
        float distance_obstacle = environment.getCollision();
        float angle_obstacle = environment.getObstacleAngle();

        //Processo de Fuzzificação------------------------------------------------------------
        //Desativa todas regras que podem ser ativadas
        bool isBall_left = false, isBall_front = false, isBall_right = false;
        bool isTarget_left = false, isTarget_front = false, isTarget_right = false;
        bool isObstacle_distance_small = false, isObstacle_distance_medium = false, isObstacle_distance_big = false;
        bool isObstacle_left = false, isObstacle_front = false, isObstacle_right = false;

        //Zera todos valores para que novos sejam calculados
        float fuzz_value_ball_left = 0, fuzz_value_ball_front = 0, fuzz_value_ball_right = 0;
        float fuzz_value_target_left = 0, fuzz_value_target_front = 0, fuzz_value_target_right = 0;
        float fuzz_value_distance_small = 0, fuzz_value_distance_medium = 0, fuzz_value_distance_big = 0;
        float fuzz_value_obstacle_left = 0, fuzz_value_obstacle_front = 0, fuzz_value_obstacle_right = 0;

        //Zera os valores de cada Regra de Esquerda, Frente e Direita
        float total_value_left_rules = 0, total_value_front_rules = 0, total_value_right_rules = 0;

        // Verifica ativação de ANGULO BOLA
        if (angles[0][0] <= angle_ball && angle_ball <= angles[0][2]) { //verifica se o valor está entre -PI e 0, se estiver pertence ao conjunto E
            isBall_left = true;
            fuzz_value_ball_left = fuzzify(angles[0][0],angles[0][1],angles[0][2],angle_ball);
        }
        if (angles[1][0] <= angle_ball && angle_ball <= angles[1][2]) { //verifica se o valor está entre -1 e 1, se estiver pertence ao conjunto F
            isBall_front = true;
            fuzz_value_ball_front = fuzzify(angles[1][0],angles[1][1],angles[1][2],angle_ball);
        }
        if (angles[2][0] <= angle_ball && angle_ball <= angles[2][2]) { //verifica se o valor está entre 0 e PI, se estiver pertence ao conjunto D
            isBall_right = true;
            fuzz_value_ball_right = fuzzify(angles[2][0],angles[2][1],angles[2][2],angle_ball);
        }

        // Verifica ativação de ANGULO ALVO
        if (angles[0][0] <= angle_target && angle_target <= angles[0][2]) { //verifica se o valor está entre -PI e 0, se estiver pertence ao conjunto E
            isTarget_left = true;
            fuzz_value_target_left = fuzzify(angles[0][0],angles[0][1],angles[0][2],angle_target);
        }
        if (angles[1][0] <= angle_target && angle_target <= angles[1][2]) { //verifica se o valor está entre -1 e 1, se estiver pertence ao conjunto F
            isTarget_front = true;
            fuzz_value_target_front = fuzzify(angles[1][0],angles[1][1],angles[1][2],angle_target);
        }
        if (angles[2][0] <= angle_target && angle_target <= angles[2][2]) { //verifica se o valor está entre 0 e PI, se estiver pertence ao conjunto D
            isTarget_right = true;
            fuzz_value_target_right = fuzzify(angles[2][0],angles[2][1],angles[2][2],angle_target);
        }

        // DISTANCIA ATE OBSTACULO
        if (distances[0][0] <= distance_obstacle && distance_obstacle <= distances[0][2]) {
             // Ativa se o obstáculo estiver a uma distância pequena
            isObstacle_distance_small = true;
            fuzz_value_distance_small = fuzzify(distances[0][0],distances[0][1],distances[0][2],distance_obstacle);
        }
        if (distances[1][0] <= distance_obstacle && distance_obstacle <= distances[1][2]) {
            // Ativa se o obstáculo estiver a uma distância média
            isObstacle_distance_medium = true;
            fuzz_value_distance_medium = fuzzify(distances[1][0],distances[1][1],distances[1][2],distance_obstacle);
        }
        if (distances[2][0] <= distance_obstacle && distance_obstacle <= distances[2][2]) {
             // Ativa se o obstáculo estiver a uma distância longa
            isObstacle_distance_big = true;
            fuzz_value_distance_big = fuzzify(distances[2][0],distances[2][1],distances[2][2],distance_obstacle);
        }

        // ANGULO OBSTACULO
        if (angles[0][0] <= angle_obstacle && angle_obstacle <= angles[0][2]) {
            // ativa se o obstaculo estiver a esquerda
            isObstacle_left = true;
            fuzz_value_obstacle_left = fuzzify(angles[0][0],angles[0][1],angles[0][2],angle_obstacle);
        }
        if (angles[1][0] <= angle_obstacle && angle_obstacle <= angles[1][2]) {
            // ativa se o obstaculo estiver a frente
            isObstacle_front = true;
            fuzz_value_obstacle_front = fuzzify(angles[1][0],angles[1][1],angles[1][2],angle_obstacle);
        }
        if (angles[2][0] <= angle_obstacle && angle_obstacle <= angles[2][2]) {
            // ativa se o obstaculo estiver a direita
            isObstacle_right = true;
            fuzz_value_obstacle_right = fuzzify(angles[2][0],angles[2][1],angles[2][2],angle_obstacle);
        }

        //Calcula valores das regras, aqui vai o cálculo do Mamdami------------------------------------------------------------
        if (isBall_left && isTarget_left) { //regra R1 ativa
            total_value_front_rules = max(min(fuzz_value_ball_left,fuzz_value_target_left), total_value_front_rules);
        }
        if (isBall_left && isTarget_front) { //regra R2 ativa
            total_value_left_rules = max(min(fuzz_value_ball_left,fuzz_value_target_front), total_value_left_rules);
        }
        if (isBall_left && isTarget_right) { //regra R3 ativa
            total_value_left_rules = max(min(fuzz_value_ball_left,fuzz_value_target_right), total_value_left_rules);
        }
        if (isBall_front && isTarget_left) { //regra R4 ativa
            total_value_right_rules = max(min(fuzz_value_ball_front,fuzz_value_target_left), total_value_right_rules);
        }
        if (isBall_front && isTarget_front) { //regra R5 ativa
            total_value_front_rules = max(min(fuzz_value_ball_front,fuzz_value_target_front), total_value_front_rules);
        }
        if (isBall_front && isTarget_right) { //regra R6 ativa
            total_value_left_rules = max(min(fuzz_value_ball_front,fuzz_value_target_right), total_value_left_rules);
        }
        if (isBall_right && isTarget_left) { //regra R7 ativa
            total_value_right_rules = max(min(fuzz_value_ball_right,fuzz_value_target_left), total_value_right_rules);
        }
        if (isBall_right && isTarget_front) { //regra R8 ativa
            total_value_right_rules = max(min(fuzz_value_ball_right,fuzz_value_target_front), total_value_right_rules);
        }
        if (isBall_right && isTarget_right) { //regra R9 ativa
            total_value_front_rules = max(min(fuzz_value_ball_right,fuzz_value_target_right), total_value_front_rules);
        }

        // Regras de Obstáculo
        // perto de obstáculo e alvo (gol) a esquerda
        if (isObstacle_distance_small && isTarget_left) {
            total_value_left_rules = max(min(fuzz_value_distance_small,fuzz_value_target_left), total_value_left_rules);
        }
        // perto de obstáculo e obstáculo a esquerda -> vira pra direita
        if (isObstacle_distance_small && isObstacle_left) {
            total_value_right_rules = max(min(isObstacle_distance_small,fuzz_value_obstacle_left), total_value_right_rules);
        }
        // perto de obstáculo e obstáculo a direita -> vira pra esquerda
        if (isObstacle_distance_small && isObstacle_right) {
            total_value_left_rules = max(min(isObstacle_distance_small,fuzz_value_obstacle_right), total_value_left_rules);
        }
        // perto de obstáculo e alvo (gol) a frente
        if (isObstacle_distance_small && isTarget_front) {
            total_value_front_rules = max(min(fuzz_value_distance_small,fuzz_value_target_front), total_value_front_rules);
        }
        // média dist de obstáculo e alvo (gol) a frente
        if (isObstacle_distance_medium && isTarget_front) {
            total_value_front_rules = max(min(fuzz_value_distance_medium,fuzz_value_target_front), total_value_front_rules);
        }

        //Processo de Defuzificação ------------------------------------------------------------------------------------------------------
        float sumY = 0;
        float sumXY = 0;

        for(float x = -PI; x <= PI; x+= 0.1)
        {
            float y = 0;
            if(angles[0][0] <= x && x <= angles[0][2])
            {
                y = max(min(fuzzify(angles[0][0],angles[0][1],angles[0][2], x), total_value_left_rules), y);
            }

            if(angles[1][0] <= x && x <= angles[1][2])
            {
                y = max(min(fuzzify(angles[1][0],angles[1][1],angles[1][2], x), total_value_front_rules), y);
            }

            if(angles[2][0] <= x && x <= angles[2][2])
            {
                y = max(min(fuzzify(angles[2][0],angles[2][1],angles[2][2], x), total_value_right_rules), y);
            }
            sumXY += x * y;
            sumY += y;
        }

        float motor = sumXY / sumY;
        motor /= 1.5;

        float leftMotor  = (cos(motor) - sin(motor)) / 2.5;
        float rightMotor = (cos(motor) + sin(motor)) / 2.5;

        if(environment.getCollision() < 100){
            if(environment.getObstacleAngle() < 45 && environment.getObstacleAngle() > -45){
                leftMotor *= -1;
                rightMotor *= -1;
            }
        }

        // Transmite ação do robô ao ambiente. Fica bloqueado até que todos os
        // robôs joguem. Se erro, retorna false (neste exemplo, sai do laco).
        if ( ! environment.act( leftMotor, rightMotor ) ) {
            break; // Termina a execução se falha ao agir.
        }
    }

    return 0;
}
//------------------------------------------------------------------------------

