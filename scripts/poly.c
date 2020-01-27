#include "stdio.h"
#include "math.h"
#include <time.h>   	// for clock_t, clock(), CLOCKS_PER_SEC
#include <unistd.h> 	// for sleep()
#define rows 16
#define cols 16


double Z[rows][cols];
double X[rows][cols];
double Y[rows][cols];

double cubic(double x, double y){
	return pow(y,3) - 10*pow(x,2) - pow(y,2) + 1;
}

double poly(double* P, double x, double y){
	return  P[0]*pow(x,3)+P[1]*pow(x,2)+P[2]*pow(x,1)+P[3]*pow(y,3)+P[4]*pow(y,2)+P[5]*pow(y,1)+P[6];
}

double residual(double* P){
	double total = 0.0;
	for (int i = 0; i<rows; i++){
		for(int j = 0; j<cols; j++){
			total += pow((poly(P, X[i][j], Y[i][j]) - Z[i][j]), 2);
		}
	}
	return total;
}

void update_grad(double* P, double* g, double gdel){
	double fx = 0.0,fdx = 0.0;
	for (int i = 0; i<7; i++){
		P[i] += gdel;
		fx = residual(P);
		P[i] -= 2*gdel;
		fdx = residual(P);
		g[i] = (fx-fdx)/(2*gdel);
		// printf("%.2lf\t",g[i]);
		P[i] += gdel;
	}
	// printf("\n");
}

void update_params(double* P, double* g, double alpha){
	double gd[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double fx = 0.0,fdx = 0.0;
	for (int i = 0; i<7; i++){
		P[i] -= alpha*g[i]; 
		// printf("%.2lf\t",g[i]);
	}
	// printf("\t");
}

void print(double* P){
	for(int i = 0; i<7; i++){
		printf("%.2lf\t",P[i]);
	}
	printf("\n");
}

int main(int argc, char** argv){
	// printf("Hello Python\n");

	double xy_min = -5.0;
	double xy_max = 5.0;
	double n = 15;
	double result = 0;


	for (int i = 0; i<rows; i++){
		for(int j = 0;j<cols; j++){
			X[i][j] = xy_min + i*(xy_max-xy_min)/n;
			Y[i][j] = xy_min + j*(xy_max-xy_min)/n;
			Z[i][j] = cubic( X[i][j], Y[i][j]);
			// printf("%.2lf\t",Z[i][j]);
		}
		// printf("\n");
	}

	double P[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double g[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double alpha = 0.000001;
	double gdel = 0.01;
	double r = 10000000.0;




	double time_spent = 0.0;
	clock_t begin = clock();

	while(1){
		r = residual(P);
		// printf("%.2lf\n",r);
		if(r > 10000){
			update_grad(P, g, gdel);
			update_params(P, g, alpha);
		}else{
			print(P);
			break;
		}
	}

	clock_t end = clock();

	time_spent += (double)(end - begin) / CLOCKS_PER_SEC;

	printf("Time elpased is %.2lf ms\n", time_spent*1000);

	return 0;
}