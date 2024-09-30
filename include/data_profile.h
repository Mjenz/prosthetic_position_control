#ifndef DATA_PROF_
#define DATA_PROF_

#define DATA_SAMPLES 3998

#define DATA_SAMPLES_REAL_7 1600
#define DATA_SAMPLES_REAL_9 1500
#define DATA_SAMPLES_REAL_11 1300
#define DATA_SAMPLES_REAL_13 1200

#define DATA_SAMPLES_REAL_cropped_7 115
#define DATA_SAMPLES_REAL_cropped_9 114
#define DATA_SAMPLES_REAL_cropped_11 102
#define DATA_SAMPLES_REAL_cropped_13 100

#define DATA_SAMPLES_REAL_adjusted_7 432
#define DATA_SAMPLES_REAL_adjusted_9 446
#define DATA_SAMPLES_REAL_adjusted_11 529
#define DATA_SAMPLES_REAL_adjusted_13 599

#define DATA_SAMPLES_10  630
#define DATA_SAMPLES_15  420
#define DATA_SAMPLES_20  316
#define DATA_SAMPLES_25  253
#define DATA_SAMPLES_30  211
#define DATA_SAMPLES_35  181
#define DATA_SAMPLES_40  159
#define DATA_SAMPLES_45  141
#define DATA_SAMPLES_50  127
#define DATA_SAMPLES_55  116
#define DATA_SAMPLES_60  106
#define DATA_SAMPLES_65  98
#define DATA_SAMPLES_70  91
#define DATA_SAMPLES_75  85
#define DATA_SAMPLES_80  80
#define DATA_SAMPLES_85  75

extern float Angle_Profile_REAL_7[],Angle_Profile_REAL_9[],Angle_Profile_REAL_11[],Angle_Profile_REAL_13[];
extern float Angle_Profile_REAL_cropped_7[],Angle_Profile_REAL_cropped_9[],Angle_Profile_REAL_cropped_11[],Angle_Profile_REAL_cropped_13[];
extern float Angle_Profile_REAL_adjusted_7[],Angle_Profile_REAL_adjusted_9[],Angle_Profile_REAL_adjusted_11[],Angle_Profile_REAL_adjusted_13[];

extern const float* angle_profiles[];
extern const float* angle_profiles_REAL[];
extern const float* angle_profiles_REAL_cropped[];
extern const float* angle_profiles_REAL_adjusted[];
extern const float* accel_profiles[];
extern const float* accel_profiles_REAL[];
extern const float* accel_profiles_REAL_cropped[];
extern const float* accel_profiles_REAL_adjusted[];
extern const int profile_lengths[],profile_lengths_REAL[],profile_lengths_REAL_cropped[],profile_lengths_REAL_adjusted[];

extern float Torque_Profile_05[DATA_SAMPLES], Torque_Profile_08[DATA_SAMPLES], Torque_Profile_10[DATA_SAMPLES],Torque_Profile_12[DATA_SAMPLES],Torque_Profile_SIN[DATA_SAMPLES];
extern float Accel_Profile_05[DATA_SAMPLES-2], Accel_Profile_08[DATA_SAMPLES-2], Accel_Profile_10[DATA_SAMPLES-2], Accel_Profile_12[DATA_SAMPLES-2], Accel_Profile_SIN[DATA_SAMPLES-2], Accel_Profile_STEP[DATA_SAMPLES-2];
extern float Angle_Profile_SIN[DATA_SAMPLES], Angle_Profile_STEP[DATA_SAMPLES];

extern float Angle_Profile_SIN_10_2_[DATA_SAMPLES_10];
extern float Accel_Profile_SIN_10_2_[DATA_SAMPLES_10-2];
extern float Angle_Profile_SIN_15_2_[DATA_SAMPLES_15];
extern float Accel_Profile_SIN_15_2_[DATA_SAMPLES_15-2];
extern float Angle_Profile_SIN_20_2_[DATA_SAMPLES_20];
extern float Accel_Profile_SIN_20_2_[DATA_SAMPLES_20-2];
extern float Angle_Profile_SIN_25_2_[DATA_SAMPLES_25];
extern float Accel_Profile_SIN_25_2_[DATA_SAMPLES_25-2];
extern float Angle_Profile_SIN_30_2_[DATA_SAMPLES_30];
extern float Accel_Profile_SIN_30_2_[DATA_SAMPLES_30-2];
extern float Angle_Profile_SIN_35_2_[DATA_SAMPLES_35];
extern float Accel_Profile_SIN_35_2_[DATA_SAMPLES_35-2];
extern float Angle_Profile_SIN_40_2_[DATA_SAMPLES_40];
extern float Accel_Profile_SIN_40_2_[DATA_SAMPLES_40-2];
extern float Angle_Profile_SIN_45_2_[DATA_SAMPLES_45];
extern float Accel_Profile_SIN_45_2_[DATA_SAMPLES_45-2];
extern float Angle_Profile_SIN_50_2_[DATA_SAMPLES_50];
extern float Accel_Profile_SIN_50_2_[DATA_SAMPLES_50-2];
extern float Angle_Profile_SIN_55_2_[DATA_SAMPLES_55];
extern float Accel_Profile_SIN_55_2_[DATA_SAMPLES_55-2];
extern float Angle_Profile_SIN_60_2_[DATA_SAMPLES_60];
extern float Accel_Profile_SIN_60_2_[DATA_SAMPLES_60-2];
extern float Angle_Profile_SIN_65_2_[DATA_SAMPLES_65];
extern float Accel_Profile_SIN_65_2_[DATA_SAMPLES_65-2];
extern float Angle_Profile_SIN_70_2_[DATA_SAMPLES_70];
extern float Accel_Profile_SIN_70_2_[DATA_SAMPLES_70-2];
extern float Angle_Profile_SIN_75_2_[DATA_SAMPLES_75];
extern float Accel_Profile_SIN_75_2_[DATA_SAMPLES_75-2];
extern float Angle_Profile_SIN_80_2_[DATA_SAMPLES_80];
extern float Accel_Profile_SIN_80_2_[DATA_SAMPLES_80-2];
extern float Angle_Profile_SIN_85_2_[DATA_SAMPLES_85];
extern float Accel_Profile_SIN_85_2_[DATA_SAMPLES_85-2];


#endif // DATA_PROF_