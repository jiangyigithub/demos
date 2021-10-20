#ifndef HUNGARIANMETHOD_H
#define HUNGARIANMETHOD_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
//#include <atlstr.h>

using namespace std;

#define HUNGARIAN_NOT_ASSIGNED 0 
#define HUNGARIAN_ASSIGNED 1

#define HUNGARIAN_MODE_MINIMIZE_COST   0
#define HUNGARIAN_MODE_MAXIMIZE_UTIL 1

typedef struct {
  int num_rows;
  int num_cols;
  int** cost;
  int** assignment;
} hungarian_problem_t;


typedef vector<double, allocator<double> >DoubleVector;
/********************************************************************
********************************************************************
**
** libhungarian by Cyrill Stachniss, 2004
**
**
** Solving the Minimum Assignment Problem using the
** Hungarian Method.
**
** ** This file may be freely copied and distributed! **
**
** Parts of the used code was originally provided by the
** "Stanford GraphGase", but I made changes to this code.
** As asked by  the copyright node of the "Stanford GraphGase",
** I hereby proclaim that this file are *NOT* part of the
** "Stanford GraphGase" distrubition!
**
** This file is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied
** warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
** PURPOSE.
**
********************************************************************
********************************************************************/

class CHungarianMethod
{
public:
  CHungarianMethod(void);
  ~CHungarianMethod(void);


  hungarian_problem_t p;
  int** costMatrix;
  int costMatrixSize;

  bool inited;
  /** This method initialize the hungarian_problem structure and init
  *  the  cost matrices (missing lines or columns are filled with 0).
  *  It returns the size of the quadratic(!) assignment matrix. **/
  int hungarian_init(hungarian_problem_t* p,
    int** cost_matrix,
    int rows,
    int cols,
    int mode);

  /** Free the memory allocated by init. **/
  void hungarian_free(hungarian_problem_t* p);

  /** This method computes the optimal assignment. **/
  bool hungarian_solve(hungarian_problem_t* p);

  /** Print the computed optimal assignment. **/
  void hungarian_print_assignment(hungarian_problem_t* p = 0);

  /** Print the cost matrix. **/
  void hungarian_print_costmatrix(hungarian_problem_t* p = 0);

  /** Print cost matrix and assignment matrix. **/
  void hungarian_print_status();

  void hungarian_print_matrix(int** C, int rows, int cols);

  int hungarian_imax(int a, int b);

  int** array_to_matrix(int* m, int rows, int cols);

  bool AssignVectorElements(DoubleVector *vec1, DoubleVector *vec2, DoubleVector *assingmentVec);
  void AllocateCostMatrix(void);
  void DeAllocateCostMatrix(void);
  bool SetCostData(int i, int j, int cost);
  bool CalcAssignment(std::vector<int> * assingmentVec);
  void InitCostMatrix(int cMSize);
  void Clear(void);
};

#endif

