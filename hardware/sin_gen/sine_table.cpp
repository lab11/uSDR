
/* sin example */
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

int main ()
{
	int freq = 80;
 	float result;
	for (int i=0; i<freq; i++){
		result = sin(2*i*PI/freq);
		result = round((result+1)*127);
		int temp = result;
	    printf ("\t\t7'd%d: out_reg = 8'h%x;\r\n", i, temp);
	}
	return 0;
}
