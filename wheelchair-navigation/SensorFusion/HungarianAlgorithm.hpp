
#pragma once
#include <iostream>
#include <vector>
#include "SensorFrame.hpp" // בהנחה שזה הקובץ המגדיר את הטיפוס

using namespace std;


class HungarianAlgorithm
{
public:
	HungarianAlgorithm();
	~HungarianAlgorithm();
	double Solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment);
    // vector<shared_ptr<SensorfusionObject>> Sensorfusion(FramePointers* fp);//בניית אוביקט מאוחד מפריים
    // vector<vector<double>> BuildingCostmatrix (vector<shared_ptr<SensorfusionObject>>* Sf,vector<shared_ptr<TrackedObject>> tr );//בניית מטריצת עלויות

private:
	void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
	void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
	void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
	void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
	void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
};


