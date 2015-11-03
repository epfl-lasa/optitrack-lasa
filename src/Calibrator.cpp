#include "Calibrator.h"
#include <iostream>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define NR_END 1
#define FREE_ARG char*
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
static double dmaxarg1,dmaxarg2;
#define DMAX(a,b) (dmaxarg1=(a),dmaxarg2=(b),(dmaxarg1) > (dmaxarg2) ?(dmaxarg1) : (dmaxarg2))
static int iminarg1,iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ?(iminarg1) : (iminarg2))

using namespace std;


Calibrator::Calibrator()
{
	vision_pose= (double**)new double[DATAPOINTSNO];
	for(int i=0; i<DATAPOINTSNO; i++)
		vision_pose[i] = new double[3];

	robot_pose= (double**)new double[DATAPOINTSNO];
	for(int i=0; i<DATAPOINTSNO; i++)
		robot_pose[i] = new double[3];
//
	rotmatrix = new double*[3];
	for(int i=0; i<3; i++)
		rotmatrix[i] = new double[3];
	transmatrix = new double[3];
//	for(int i=0; i<pose_count; i++)
//		for(int j=0; j<3; j++)
//			MatrixVtR[i][j] = 0;

	pose_count = 0;
//	for(int i =0; i<pose_count; i++)
//		for(int j=0; j<3; j++)
//			MatrixVtR[i][j] = 0;

}

void Calibrator::resetCalibration()
{
//	free(vision_pose);
//	free(robot_pose);

	pose_count = 0;
}


bool Calibrator::addCalibrationPoint(double *robot_ee_pose, double *vision_ee_pose)
{
	if(vision_ee_pose[0] ==0 && vision_ee_pose[1] ==0 && vision_ee_pose[2] == 0)
		{
			cout<<"\nInvalid Vision position (0,0,0) captured.\nData point not added!"<<endl;
			return false;
		}
	if(robot_ee_pose[0] ==0 && robot_ee_pose[1] == 0 && robot_ee_pose[2] == 0)
		{
			cout<<"\nInvalid Robot position (0,0,0) captured.\nData point not added!"<<endl;
			return false;
		}
	if(pose_count>= DATAPOINTSNO)
	{
		cout<<"\nMaximum no. of calibration points reached ("<<DATAPOINTSNO<<").\nNo further points can be added";
		return false;
	}

	for(int i=0; i<3; i++)
			robot_pose[pose_count][i] = robot_ee_pose[i];
		for(int i=0; i<3; i++)
			vision_pose[pose_count][i] = vision_ee_pose[i];

		cout<<"\nAdded points:"<<pose_count+1<<"\n";
		cout<<robot_pose[pose_count][0]<<" "<<robot_pose[pose_count][1]<<" "<<robot_pose[pose_count][2]<<endl;
		cout<<vision_pose[pose_count][0]<<" "<<vision_pose[pose_count][1]<<" "<<vision_pose[pose_count][2]<<endl;

		pose_count++;
		return true;
}

//bool Calibrator::addCalibrationPoint(double **robot_matrix, double *vision_ee_pose)
//{
//	for(int i=0; i<3; i++)
//		for(int j=0; j<3; j++)
//			MatrixEtR[i][j] = robot_matrix[i][j];
//	vision_pose = vision_ee_pose;
//	return true;
//}


bool Calibrator::solveCalibration(double **rotation, double *trans)
{
	double **matrixR= new double*[3];
	for(int i=0; i<3; i++)
		matrixR[i] = new double[pose_count];
	double **matrixV= new double*[3];
	for(int i=0; i<3; i++)
		matrixV[i] = new double[pose_count];

//	double *matrixV= (double**)new double[3][pose_count];
	double robot_avg[3] = {0.0, 0.0, 0.0};
	double vision_avg[3] = {0.0, 0.0, 0.0};

	for(int i =0; i<pose_count; i++)
		for(int j=0; j<3; j++)
		{
			robot_avg[j] += robot_pose[i][j];
			vision_avg[j] += vision_pose[i][j];
		}

	for(int i = 0; i<3; i++)
	{
		robot_avg[i] = robot_avg[i]/pose_count;
		vision_avg[i] = vision_avg[i]/pose_count;
	}

	double **vision_c_pose = new double*[DATAPOINTSNO];
	double **robot_c_pose = new double*[DATAPOINTSNO];
	for(int i=0; i<DATAPOINTSNO; i++)
	{
		vision_c_pose[i] = new double[3];
		robot_c_pose[i] = new double[3];
	}



	for(int i= 0; i<pose_count; i++)
		for(int j=0; j<3; j++)
		{
			robot_c_pose[i][j] = robot_pose[i][j] - robot_avg[j];
			vision_c_pose[i][j] = vision_pose[i][j] - vision_avg[j];
		}


	MatrixTranspose(robot_c_pose, matrixR, pose_count, 3);
	MatrixTranspose(vision_c_pose, matrixV, pose_count, 3);


	double **H = new double*[3];
	for(int i=0; i<3; i++)
		H[i] = new double[3];



	MatrixMultipy(matrixV, robot_c_pose, H, 3, 3, pose_count);


	double w[4];
	double **V1 = new double*[4];
	double **V =  new double*[3];
	double **Ht = new double*[3];
	double **temp= new double*[3];
	for(int i =0; i<3; i++)
	{
		V[i] = new double[3];
		Ht[i] = new double[3];
		temp[i] = new double[1];
	}
	MatrixTranspose(H, Ht, 3, 3);
	double **Htemp = new double*[4];
	for(int i=0; i<4; i++)
	{
		Htemp[i] = new double[4];
		V1[i] = new double[4];

	}

	for(int i=1; i<4; i++)
		for(int j = 1; j<4; j++)
	{
		Htemp[i][j] = H[i-1][j-1];
	}

	Htemp[0][0] = Htemp[0][1] = Htemp[0][2] = Htemp[0][3] = Htemp[1][0] = Htemp[2][0] = Htemp[3][0] = 0;

	svdcmp(Htemp, 3, 3, w, V1);


	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			H[i][j] = Htemp[i+1][j+1];
			V[i][j] = V1[i+1][j+1];
		}

//	for(int i=0; i<3; i++)
//	{
//		V[i][1] *= -1;
//		H[i][1] *= -1;
// 	}

	MatrixTranspose(H, Ht, 3, 3);

	MatrixMultipy(V, Ht, rotation, 3,3,3);

	double *temp2 = new double[3];

	for (int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			temp2[i] += rotation[i][j]*vision_avg[j] ;
		}

	for (int i=0; i<3; i++)
		trans[i] = robot_avg[i] - temp2[i];
//	rotmatrix = rotation;
	cout<<"Rot: "<<endl;
	for(int i =0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			rotmatrix[i][j] = rotation[i][j];
			cout<<rotmatrix[i][j]<<"  ";
		}
		cout<<endl;
	}
	cout<<"Trans: "<<endl;
	for(int i=0; i<3; i++)
	{
			transmatrix[i] = trans[i];
			cout<<transmatrix[i]<<"  ";
	}
	cout<<endl;

	free(matrixR);
	free(matrixV);
	return true;
}

void Calibrator::saveCalibration(const char *filename)
{
//	filename = "/home/saurav/calibresult.txt";
	FILE *calib = fopen(filename, "w");
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			fprintf(calib,"%lf \t", rotmatrix[i][j]);

		}
		fprintf(calib, "\n");

	}
	for(int i=0; i<3; i++)
		fprintf(calib, "%lf \t", transmatrix[i]);
	fclose(calib);
	cout<<"Calibration saved to "<<filename<<endl;

}


void Calibrator::gettransformation(double **rot, double *tran)
{
	for(int i=0; i<3; i++)
		{
		for(int j=0; j<3; j++)
				rot[i][j] = rotmatrix[i][j];
		tran[i] = transmatrix[i];
		}

}

void Calibrator::recordData()
{
	FILE *data = fopen("calibdata.txt", "w");
	for(int i=0; i<pose_count; i++)
	{
		fprintf(data, "%lf %lf %lf ", robot_pose[i][0], robot_pose[i][1], robot_pose[i][2]);
		fprintf(data, "%lf %lf %lf \n", vision_pose[i][0], vision_pose[i][1], vision_pose[i][2]);
	}
	fclose(data);
}

void Calibrator::PrintCalibError()

{
	//temp = R*vision_pose
	double **error = new double*[3];
	for(int j=0; j<3; j++)
		error[j] = new double[DATAPOINTSNO];

	float max_error = 0.0;
	float mean_error[3] = {0.0};

	double **temp = new double*[3];
	for(int j=0; j<3; j++)
		temp[j] = new double[DATAPOINTSNO];
//	cout<<"\nPose Count="<<pose_count<<endl;

	MatrixTranspose(vision_pose, temp, pose_count, 3);
//	MatrixPrint(temp, 3, pose_count);
	cout<<endl;
	MatrixMultipy(rotmatrix, temp, error, 3, pose_count, 3);

//	cout<<"Rotation Matrix:\n";

//	MatrixPrint(rotmatrix, 3,3);

	for(int i=0; i<pose_count; i++)
		for(int j=0; j<3; j++)
			{
			error[j][i] += transmatrix[j] - robot_pose[i][j];
			if(error[j][i]> max_error)
				max_error = error[j][i];
			mean_error[j] += error[j][i]/pose_count;


			}

		cout<<"\nMean error: ";
			for(int j=0; j<3; j++)
				cout<<mean_error[j]<<"\t";
			cout<<endl;
			cout<<"\nMax error:"<<max_error<<endl;

//	MatrixPrint(robot_pose, pose_count, 3);

}
//this function is based on the website
//http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche52.html

//void Calibrator::MatrixfromPose(double** matrix, double* pose)
//{
//	matrix[3][0] = matrix[3][1] = matrix[3][2]= 0;
//	matrix[3][3] = 1;
//	matrix[0][3] = pose[0];		//equating x
//	matrix[1][3] = pose[1];		//equating y
//	matrix[2][3] = pose[2];		//equating z
//	matrix[0][0] = pose[3]*pose[3] + pose[4]*pose[4] - pose[5]*pose[5] - pose[6]*pose[6] ;
//	matrix[0][1] = 2*(pose[4]*pose[5] - pose[3]*pose[6]);
//	matrix[0][2] = 2*(pose[4]*pose[6] + pose[3]*pose[5]);
//	matrix[1][0] = 2*(pose[4]*pose[5] + pose[3]*pose[6]);
//	matrix[1][1] = pose[3]*pose[3] - pose[4]*pose[4] + pose[5]*pose[5] - pose[6]*pose[6] ;
//	matrix[1][2] = 2*(pose[5]*pose[6] - pose[3]*pose[4]);
//	matrix[2][0] = 2*(pose[4]*pose[6] - pose[3]*pose[5]);
//	matrix[2][1] = 2*(pose[5]*pose[6] + pose[3]*pose[4]);
//	matrix[2][2] = pose[3]*pose[3] - pose[4]*pose[4] - pose[5]*pose[5] + pose[6]*pose[6] ;
//}
//
//void Calibrator::InvertMatrix(double** matrix, double** invmatrix)
//{
//	invmatrix[3][0] = invmatrix[3][1] = invmatrix[3][2] = 0;
//	invmatrix[3][3] = 1;
//	invmatrix[0][3] = -(matrix[0][0]*matrix[0][3] + matrix[1][0]*matrix[1][3] + matrix[2][0]*matrix[2][3]);
//	invmatrix[1][3] = -(matrix[0][1]*matrix[0][3] + matrix[1][1]*matrix[1][3] + matrix[2][1]*matrix[2][3]);
//	invmatrix[2][3] = -(matrix[0][2]*matrix[0][3] + matrix[1][2]*matrix[1][3] + matrix[2][2]*matrix[2][3]);
//	for(int i =0; i<3; i++)		//transposing rotation matrix
//		for(int j=0; j<3; j++)
//			if(i == j)
//				invmatrix[i][j] = matrix[i][j];
//			else
//				invmatrix[i][j] = matrix[j][i];
//
//}

// matrixA is mXp matrixB is nXp Result is mXn
void Calibrator::MatrixMultipy(double **MatrixA, double **MatrixB, double **Result, int m, int n, int p)
{
	for(int i=0; i<m; i++)
		for(int j=0; j<n; j++)
		{
			Result[i][j] = 0;
			for(int k=0; k<p; k++)
			{
				Result[i][j] += MatrixA[i][k]*MatrixB[k][j];
			}
		}
}

void Calibrator::MatrixTranspose(double **matrix, double **transpose, int row, int column)
{
	for( int i= 0; i< row; i++)
		for(int j=0; j<column; j++)
			transpose[j][i] = matrix[i][j];
}

//void Calibrator::MatrixInverse(double ** matrix, int order, double** invmatrix)
//{
////	void MatrixInversion(float **A, int order, float **Y)
////	{
//	    // get the determinant of a
//	    double det = 1.0/CalcDeterminant(matrix,order);
//
//	    // memory allocation
//	    double *temp = new double[(order-1)*(order-1)];
//	    double **minor = new double*[order-1];
//	    for(int i=0;i<order-1;i++)
//	        minor[i] = temp+(i*(order-1));
//
//	    for(int j=0;j<order;j++)
//	    {
//	        for(int i=0;i<order;i++)
//	        {
//	            // get the co-factor (matrix) of A(j,i)
//	            GetMinor(matrix,minor,j,i,order);
//	            invmatrix[i][j] = det*CalcDeterminant(minor,order-1);
//	            if( (i+j)%2 == 1)
//	                invmatrix[i][j] = -invmatrix[i][j];
//	        }
//	    }
//
//}

//int Calibrator::GetMinor(double **src, double **dest, int row, int col, int order)
//{
//	    // indicate which col and row is being copied to dest
//	    int colCount=0;
//		int rowCount=0;
//
//	    for(int i = 0; i < order; i++ )
//	    {
//	        if( i != row )
//	        {
//	            colCount = 0;
//	            for(int j = 0; j < order; j++ )
//	            {
//	                // when j is not the element
//	                if( j != col )
//	                {
//	                    dest[rowCount][colCount] = src[i][j];
//	                    colCount++;
//	                }
//	            }
//	            rowCount++;
//	        }
//	    }
//
//	    return 1;
//}
//
//double Calibrator::CalcDeterminant( double **mat, int order)
//{
//    // order must be >= 0
//	// stop the recursion when matrix is a single element
//    if( order == 1 )
//        return mat[0][0];
//
//    // the determinant value
//    double det = 0;
//
//    // allocate the cofactor matrix
//    double **minor;
//    minor = new double*[order-1];
//    for(int i=0;i<order-1;i++)
//        minor[i] = new double[order-1];
//
//    for(int i = 0; i < order; i++ )
//    {
//        // get minor of element (0,i)
//        GetMinor( mat, minor, 0, i , order);
//        // the recusion is here!
//
//        det += (i%2==1?-1.0:1.0) * mat[0][i] * CalcDeterminant(minor,order-1);
//        //det += pow( -1.0, i ) * mat[0][i] * CalcDeterminant( minor,order-1 );
//    }
//
//    // release memory
//    for(int i=0;i<order-1;i++)
//        delete [] minor[i];
//    delete [] minor;
//
//    return det;
//}


void Calibrator::MatrixPrint(double ** matrix, int row, int column)
{
	for(int i=0; i<row; i++)
	{
		for(int j=0; j<column; j++)
			cout<<matrix[i][j]<<"\t";
	cout<<endl;
	}
}




/*******************************************************************************
Singular value decomposition program, svdcmp, from "Numerical Recipes in C"
(Cambridge Univ. Press) by W.H. Press, S.A. Teukolsky, W.T. Vetterling,
and B.P. Flannery
*******************************************************************************/


double **Calibrator::dmatrix(int nrl, int nrh, int ncl, int nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	int i,nrow=nrh-nrl+1,ncol=nch-ncl+1;
	double **m;
	/* allocate pointers to rows */
	m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
	m += NR_END;
	m -= nrl;
	/* allocate rows and set pointers to them */
	m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
	m[nrl] += NR_END;
	m[nrl] -= ncl;
	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;
	/* return pointer to array of pointers to rows */
	return m;
}

double *Calibrator::dvector(int nl, int nh)
/* allocate a double vector with subscript range v[nl..nh] */
{
	double *v;
	v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
	return v-nl+NR_END;
}

void Calibrator::free_dvector(double *v, int nl, int nh)
/* free a double vector allocated with dvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

double Calibrator::pythag(double a, double b)
/* compute (a2 + b2)^1/2 without destructive underflow or overflow */
{
	double absa,absb;
	absa=fabs(a);
	absb=fabs(b);
	if (absa > absb) return absa*sqrt(1.0+(absb/absa)*(absb/absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+(absa/absb)*(absa/absb)));
}

/******************************************************************************/
void Calibrator::svdcmp(double **a, int m, int n, double w[], double **v)
/*******************************************************************************
Given a matrix a[1..m][1..n], this routine computes its singular value
decomposition, A = U.W.VT.  The matrix U replaces a on output.  The diagonal
matrix of singular values W is output as a vector w[1..n].  The matrix V (not
the transpose VT) is output as v[1..n][1..n].
*******************************************************************************/
{
	int flag,i,its,j,jj,k,l=0,nm;
	double anorm,c,f,g,h,s,scale,x,y,z,*rv1;

	rv1=dvector(1,n);
	g=scale=anorm=0.0; /* Householder reduction to bidiagonal form */
	for (i=1;i<=n;i++) {
		l=i+1;
		rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) {
			for (k=i;k<=m;k++) scale += fabs(a[k][i]);
			if (scale) {
				for (k=i;k<=m;k++) {
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				for (j=l;j<=n;j++) {
					for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
				}
				for (k=i;k<=m;k++) a[k][i] *= scale;
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if (i <= m && i != n) {
			for (k=l;k<=n;k++) scale += fabs(a[i][k]);
			if (scale) {
				for (k=l;k<=n;k++) {
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
				for (j=l;j<=m;j++) {
					for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
					for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
				}
				for (k=l;k<=n;k++) a[i][k] *= scale;
			}
		}
		anorm = DMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
	}
	for (i=n;i>=1;i--) { /* Accumulation of right-hand transformations. */
		if (i < n) {
			if (g) {
				for (j=l;j<=n;j++) /* Double division to avoid possible underflow. */
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) {
					for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=rv1[i];
		l=i;
	}
	for (i=IMIN(m,n);i>=1;i--) { /* Accumulation of left-hand transformations. */
		l=i+1;
		g=w[i];
		for (j=l;j<=n;j++) a[i][j]=0.0;
		if (g) {
			g=1.0/g;
			for (j=l;j<=n;j++) {
				for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
			}
			for (j=i;j<=m;j++) a[j][i] *= g;
		} else for (j=i;j<=m;j++) a[j][i]=0.0;
		++a[i][i];
	}
	for (k=n;k>=1;k--) { /* Diagonalization of the bidiagonal form. */
		for (its=1;its<=30;its++) {
			flag=1;
			for (l=k;l>=1;l--) { /* Test for splitting. */
				nm=l-1; /* Note that rv1[1] is always zero. */
				if ((double)(fabs(rv1[l])+anorm) == anorm) {
					flag=0;
					break;
				}
				if ((double)(fabs(w[nm])+anorm) == anorm) break;
			}
			if (flag) {
				c=0.0; /* Cancellation of rv1[l], if l > 1. */
				s=1.0;
				for (i=l;i<=k;i++) {
					f=s*rv1[i];
					rv1[i]=c*rv1[i];
					if ((double)(fabs(f)+anorm) == anorm) break;
					g=w[i];
					h=pythag(f,g);
					w[i]=h;
					h=1.0/h;
					c=g*h;
					s = -f*h;
					for (j=1;j<=m;j++) {
						y=a[j][nm];
						z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) { /* Convergence. */
				if (z < 0.0) { /* Singular value is made nonnegative. */
					w[k] = -z;
					for (j=1;j<=n;j++) v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30) printf("no convergence in 30 svdcmp iterations");
			x=w[l]; /* Shift from bottom 2-by-2 minor. */
			nm=k-1;
			y=w[nm];
			g=rv1[nm];
			h=rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=pythag(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.0; /* Next QR transformation: */
			for (j=l;j<=nm;j++) {
				i=j+1;
				g=rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=1;jj<=n;jj++) {
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag(f,h);
				w[j]=z; /* Rotation can be arbitrary if z = 0. */
				if (z) {
					z=1.0/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=1;jj<=m;jj++) {
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			rv1[l]=0.0;
			rv1[k]=f;
			w[k]=x;
		}
	}
	free_dvector(rv1,1,n);
}
