#ifndef _LOCATION_H_
#define _LOCATION_H_

/*               phS2
            ************  
				phS4************phS6 
            ************ 
            ************ 
            ************ 
            ************ 
            ************ 
				phS3************phS5 						
            ************
								 phS1
				Initialized Position
*/

#define  Start_Point  0
#define  Strai_Road1  1
#define  Curve_Road1  2
#define  Strai_Road2  3
#define  Curve_Road2  4
#define  Strai_Road3  5
#define  End_Point    6

#define  phS_ON  0   //¼ì²âµ½
#define  phS_OFF 1   //Î´¼ì²âµ½

typedef struct{
	short pre_phS1;
	short cur_phS1;
	
	short pre_phS2;
	short cur_phS2;
	
	short cur_phS3;
	short cur_phS4;
	short cur_phS5;
	short cur_phS6;
}phSensor_TypeDef;

void Road_Init(void);
void Location_Switch(void);
unsigned char Get_Road_State(void);

#endif
