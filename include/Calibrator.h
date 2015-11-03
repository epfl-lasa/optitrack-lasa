#ifndef _CALIBRATOR_H_
#define _CALIBRATOR_H_

#define DATAPOINTSNO 1000


class Calibrator
{
private:
	double **rotmatrix;
	double *transmatrix;
//	double transform[4][3];
//	double MatrixEtV[4][4];
//	double MatrixEtR[4][4];
	double **robot_pose;
	double **vision_pose;

	int pose_count;

public:
	Calibrator();

	bool addCalibrationPoint(double* robot_ee_pose, double* vision_pose);
	bool solveCalibration(double **rotation, double *trans);
	void saveCalibration(const char* filename);
	void gettransformation(double **rotation, double *translation);
	void resetCalibration();
	void recordData();
//	void MatrixfromPose(double** matrix, double* pose);
//	void InvertMatrix(double** origmatrix, double** invmatrix);
	void MatrixMultipy(double **MatrixA, double **MatrixB, double **Result, int m, int n, int p);
	void MatrixPrint(double **matrix, int row, int column);

	void PrintCalibError();
	void MatrixTranspose(double **matrix, double **transpose, int row, int column);
//	void MatrixInverse(double ** matrix, int order, double** invmatrix);
//	int GetMinor(double **src, double **dest, int row, int col, int order);
//	double CalcDeterminant( double **mat, int order);
	void svdcmp(double **a, int m, int n, double w[], double **v);

	double **dmatrix(int nrl, int nrh, int ncl, int nch);
	double *dvector(int nl, int nh);
	void free_dvector(double *v, int nl, int nh);
	double pythag(double a, double b);

//	bool addCalibrationPoint(double **robot_matrix, double *vision_ee_pose);

};



#endif
