#include "../lib/hungarian_method/include/HungarianMethod.h"

#include <cmath>
#include <iostream>


#define INF (0x7FFFFFFF)
#define verbose (0)
#define printfile (1)
#define hungarian_test_alloc(X) do {if ((void *)(X) == NULL) fprintf(stderr, "Out of memory in %s, (%s, line %d).\n", __FUNCTION__, __FILE__, __LINE__); } while (0)


CHungarianMethod::CHungarianMethod(void)
{
  inited = false;
  costMatrixSize = 0;
  costMatrix = 0;
}

void CHungarianMethod::Clear(void)
{
  if (inited) {
    hungarian_free(&p);
    inited = false;
  }

  DeAllocateCostMatrix();
}

CHungarianMethod::~CHungarianMethod(void)
{
  Clear();
}




void CHungarianMethod::hungarian_print_matrix(int** C, int rows, int cols) {

  FILE *f;
  if (printfile) {
    //CString s;
    //s.Format("0matrix_%d.txt",rand());
    char s[120];
    //sprintf_s(s, 120, "0matrix_%d.txt", rand());
    f = fopen(s, "wt");
  }




  int i, j;
  fprintf(stderr, "\n");
  for (i = 0; i<rows; i++) {
    fprintf(stderr, " [");
    for (j = 0; j<cols; j++) {
      fprintf(stderr, "%5d ", C[i][j]);
      if (printfile)
        fprintf(f, "%5d ", C[i][j]);
    }
    fprintf(stderr, "]\n");
    if (printfile)
      fprintf(f, "\n");
  }
  fprintf(stderr, "\n");
  if (printfile)
    fclose(f);

}

void CHungarianMethod::hungarian_print_assignment(hungarian_problem_t* pt) {
  hungarian_print_matrix(p.assignment, p.num_rows, p.num_cols);//(pt->assignment, pt->num_rows, pt->num_cols) ;
}

void CHungarianMethod::hungarian_print_costmatrix(hungarian_problem_t* pt) {
  //hungarian_print_matrix(pt->cost, pt->num_rows, pt->num_cols) ;
  hungarian_print_matrix(p.cost, p.num_rows, p.num_cols);
}

void CHungarianMethod::hungarian_print_status() {

  fprintf(stderr, "cost:\n");
  hungarian_print_costmatrix(&p);

  fprintf(stderr, "assignment:\n");
  hungarian_print_assignment(&p);

}

int CHungarianMethod::hungarian_imax(int a, int b) {
  return (a<b) ? b : a;
}

int CHungarianMethod::hungarian_init(hungarian_problem_t* p, int** cost_matrix, int rows, int cols, int mode) {

  int i, j, org_cols, org_rows;
  int max_cost;
  max_cost = 0;

  org_cols = cols;
  org_rows = rows;

  // is the number of cols  not equal to number of rows ? 
  // if yes, expand with 0-cols / 0-cols
  rows = hungarian_imax(cols, rows);
  cols = rows;

  p->num_rows = rows;
  p->num_cols = cols;

  p->cost = (int**)calloc(rows, sizeof(int*));
  hungarian_test_alloc(p->cost);
  p->assignment = (int**)calloc(rows, sizeof(int*));
  hungarian_test_alloc(p->assignment);

  for (i = 0; i<p->num_rows; i++) {
    p->cost[i] = (int*)calloc(cols, sizeof(int));
    hungarian_test_alloc(p->cost[i]);
    p->assignment[i] = (int*)calloc(cols, sizeof(int));
    hungarian_test_alloc(p->assignment[i]);
    for (j = 0; j<p->num_cols; j++) {
      p->cost[i][j] = (i < org_rows && j < org_cols) ? cost_matrix[i][j] : 0;
      p->assignment[i][j] = 0;

      if (max_cost < p->cost[i][j])
        max_cost = p->cost[i][j];
    }
  }


  if (mode == HUNGARIAN_MODE_MAXIMIZE_UTIL) {
    for (i = 0; i<p->num_rows; i++) {
      for (j = 0; j<p->num_cols; j++) {
        p->cost[i][j] = max_cost - p->cost[i][j];
      }
    }
  }
  else if (mode == HUNGARIAN_MODE_MINIMIZE_COST) {
    // nothing to do
  }
  else
    fprintf(stderr, "%s: unknown mode. Mode was set to HUNGARIAN_MODE_MINIMIZE_COST !\n", __FUNCTION__);

  return rows;
}




void CHungarianMethod::hungarian_free(hungarian_problem_t* p) {
  int i;
  for (i = 0; i<p->num_rows; i++) {
    free(p->cost[i]);
    free(p->assignment[i]);
  }
  free(p->cost);
  free(p->assignment);
  p->cost = NULL;
  p->assignment = NULL;
}



bool CHungarianMethod::hungarian_solve(hungarian_problem_t* p)
{
  int i, j, m, n, k, l, s, t, q, unmatched, cost;
  int* col_mate;
  int* row_mate;
  int* parent_row;
  int* unchosen_row;
  int* row_dec;
  int* col_inc;
  int* slack;
  int* slack_row;

  cost = 0;
  m = p->num_rows;
  n = p->num_cols;

  col_mate = (int*)calloc(p->num_rows, sizeof(int));
  hungarian_test_alloc(col_mate);
  unchosen_row = (int*)calloc(p->num_rows, sizeof(int));
  hungarian_test_alloc(unchosen_row);
  row_dec = (int*)calloc(p->num_rows, sizeof(int));
  hungarian_test_alloc(row_dec);
  slack_row = (int*)calloc(p->num_rows, sizeof(int));
  hungarian_test_alloc(slack_row);

  row_mate = (int*)calloc(p->num_cols, sizeof(int));
  hungarian_test_alloc(row_mate);
  parent_row = (int*)calloc(p->num_cols, sizeof(int));
  hungarian_test_alloc(parent_row);
  col_inc = (int*)calloc(p->num_cols, sizeof(int));
  hungarian_test_alloc(col_inc);
  slack = (int*)calloc(p->num_cols, sizeof(int));
  hungarian_test_alloc(slack);

  for (i = 0; i<p->num_rows; i++) {
    col_mate[i] = 0;
    unchosen_row[i] = 0;
    row_dec[i] = 0;
    slack_row[i] = 0;
  }
  for (j = 0; j<p->num_cols; j++) {
    row_mate[j] = 0;
    parent_row[j] = 0;
    col_inc[j] = 0;
    slack[j] = 0;
  }

  for (i = 0; i<p->num_rows; ++i)
    for (j = 0; j<p->num_cols; ++j)
      p->assignment[i][j] = HUNGARIAN_NOT_ASSIGNED;

  // Begin subtract column minima in order to start with lots of zeroes 12
  if (verbose)
    fprintf(stderr, "Using heuristic\n");
  for (l = 0; l<n; l++)
  {
    s = p->cost[0][l];
    for (k = 1; k<m; k++)
      if (p->cost[k][l]<s)
        s = p->cost[k][l];
    cost += s;
    if (s != 0)
      for (k = 0; k<m; k++)
        p->cost[k][l] -= s;
  }
  // End subtract column minima in order to start with lots of zeroes 12

  // Begin initial state 16
  t = 0;
  for (l = 0; l<n; l++)
  {
    row_mate[l] = -1;
    parent_row[l] = -1;
    col_inc[l] = 0;
    slack[l] = INF;
  }
  for (k = 0; k<m; k++)
  {
    s = p->cost[k][0];
    for (l = 1; l<n; l++)
      if (p->cost[k][l]<s)
        s = p->cost[k][l];
    row_dec[k] = s;
    for (l = 0; l<n; l++)
      if (s == p->cost[k][l] && row_mate[l]<0)
      {
        col_mate[k] = l;
        row_mate[l] = k;
        if (verbose)
          fprintf(stderr, "matching col %d==row %d\n", l, k);
        goto row_done;
      }
    col_mate[k] = -1;
    if (verbose)
      fprintf(stderr, "node %d: unmatched row %d\n", t, k);
    unchosen_row[t++] = k;
  row_done:
    ;
  }
  // End initial state 16

  // Begin Hungarian algorithm 18
  if (t == 0)
    goto done;
  unmatched = t;
  while (1)
  {
    if (verbose)
      fprintf(stderr, "Matched %d rows.\n", m - t);
    q = 0;
    while (1)
    {
      while (q<t)
      {
        // Begin explore node q of the forest 19
        {
          k = unchosen_row[q];
          s = row_dec[k];
          for (l = 0; l<n; l++)
            if (slack[l])
            {
              int del;
              del = p->cost[k][l] - s + col_inc[l];
              if (del<slack[l])
              {
                if (del == 0)
                {
                  if (row_mate[l]<0)
                    goto breakthru;
                  slack[l] = 0;
                  parent_row[l] = k;
                  if (verbose)
                    fprintf(stderr, "node %d: row %d==col %d--row %d\n",
                      t, row_mate[l], l, k);
                  unchosen_row[t++] = row_mate[l];
                }
                else
                {
                  slack[l] = del;
                  slack_row[l] = k;
                }
              }
            }
        }
        // End explore node q of the forest 19
        q++;
      }

      // Begin introduce a new zero into the matrix 21
      s = INF;
      for (l = 0; l<n; l++)
        if (slack[l] && slack[l]<s)
          s = slack[l];
      for (q = 0; q<t; q++)
        row_dec[unchosen_row[q]] += s;
      for (l = 0; l<n; l++)
        if (slack[l])
        {
          slack[l] -= s;
          if (slack[l] == 0)
          {
            // Begin look at a new zero 22
            k = slack_row[l];
            if (verbose)
              fprintf(stderr,
                "Decreasing uncovered elements by %d produces zero at [%d,%d]\n",
                s, k, l);
            if (row_mate[l]<0)
            {
              for (j = l + 1; j<n; j++)
                if (slack[j] == 0)
                  col_inc[j] += s;
              goto breakthru;
            }
            else
            {
              parent_row[l] = k;
              if (verbose)
                fprintf(stderr, "node %d: row %d==col %d--row %d\n", t, row_mate[l], l, k);
              unchosen_row[t++] = row_mate[l];
            }
            // End look at a new zero 22
          }
        }
        else
          col_inc[l] += s;
      // End introduce a new zero into the matrix 21
    }
  breakthru:
    // Begin update the matching 20
    if (verbose)
      fprintf(stderr, "Breakthrough at node %d of %d!\n", q, t);
    while (1)
    {
      j = col_mate[k];
      col_mate[k] = l;
      row_mate[l] = k;
      if (verbose)
        fprintf(stderr, "rematching col %d==row %d\n", l, k);
      if (j<0)
        break;
      k = parent_row[j];
      l = j;
    }
    // End update the matching 20
    if (--unmatched == 0)
      goto done;
    // Begin get ready for another stage 17
    t = 0;
    for (l = 0; l<n; l++)
    {
      parent_row[l] = -1;
      slack[l] = INF;
    }
    for (k = 0; k<m; k++)
      if (col_mate[k]<0)
      {
        if (verbose)
          fprintf(stderr, "node %d: unmatched row %d\n", t, k);
        unchosen_row[t++] = k;
      }
    // End get ready for another stage 17
  }
done:

  // Begin doublecheck the solution 23
  for (k = 0; k<m; k++)
    for (l = 0; l<n; l++)
      if (p->cost[k][l]<row_dec[k] - col_inc[l]) {
        free(slack);
        free(col_inc);
        free(parent_row);
        free(row_mate);
        free(slack_row);
        free(row_dec);
        free(unchosen_row);
        free(col_mate);
        return false;//exit(0);
      }
  for (k = 0; k<m; k++)
  {
    l = col_mate[k];
    if (l<0 || p->cost[k][l] != row_dec[k] - col_inc[l]) {
      free(slack);
      free(col_inc);
      free(parent_row);
      free(row_mate);
      free(slack_row);
      free(row_dec);
      free(unchosen_row);
      free(col_mate);
      return false;//exit(0);
    }
  }
  k = 0;
  for (l = 0; l<n; l++)
    if (col_inc[l])
      k++;
  if (k>m) {
    free(slack);
    free(col_inc);
    free(parent_row);
    free(row_mate);
    free(slack_row);
    free(row_dec);
    free(unchosen_row);
    free(col_mate);
    return false;//exit(0);
  }
  // End doublecheck the solution 23
  // End Hungarian algorithm 18

  for (i = 0; i<m; ++i)
  {
    p->assignment[i][col_mate[i]] = HUNGARIAN_ASSIGNED;
    /*TRACE("%d - %d\n", i, col_mate[i]);*/
  }
  for (k = 0; k<m; ++k)
  {
    for (l = 0; l<n; ++l)
    {
      /*TRACE("%d ",p->cost[k][l]-row_dec[k]+col_inc[l]);*/
      p->cost[k][l] = p->cost[k][l] - row_dec[k] + col_inc[l];
    }
    /*TRACE("\n");*/
  }
  for (i = 0; i<m; i++)
    cost += row_dec[i];
  for (i = 0; i<n; i++)
    cost -= col_inc[i];
  if (verbose)
    fprintf(stderr, "Cost is %d\n", cost);


  free(slack);
  free(col_inc);
  free(parent_row);
  free(row_mate);
  free(slack_row);
  free(row_dec);
  free(unchosen_row);
  free(col_mate);

  return true;
}

int** CHungarianMethod::array_to_matrix(int* m, int rows, int cols) {
  int i, j;
  int** r;
  r = (int**)calloc(rows, sizeof(int*));
  for (i = 0; i<rows; i++)
  {
    r[i] = (int*)calloc(cols, sizeof(int));
    for (j = 0; j<cols; j++)
      r[i][j] = m[i*cols + j];
  }
  return r;
}


bool CHungarianMethod::AssignVectorElements(DoubleVector *vec1, DoubleVector *vec2, DoubleVector *assingmentVec)
{

  int size = (vec1->size() >= vec2->size()) ? (vec1->size()) : (vec2->size());

  std::vector<int> distvector(size*size, 0);
  int *dist = &distvector.front();
  //int* dist = (int*) malFIXEDloc(size*size*sizeof(int));
  //memset(dist, 0, size*size*sizeof(int));

  for (int j = 0; j < vec1->size(); j++) {
    for (int i = 0; i < vec2->size(); i++) {
      dist[j*size + i] = (int)(abs(vec1->at(j) - vec2->at(i))*1000.0);

      //if (Inside(grzones[j], estpts[i]))
      //dist[j*size+i] = 1;
      //else
      //dist[j*size+i] = 0;
    }
  }

  int** m = array_to_matrix(dist, size, size);

  if (inited)
    hungarian_free(&p);
  int matrix_size = hungarian_init(&p, m, size, size, HUNGARIAN_MODE_MINIMIZE_COST);
  inited = true;

  printf("Before solve\n");
  hungarian_print_costmatrix(&p);

  //hungarian_solve(&p);
  bool ifsolved = hungarian_solve(&p);

  if (!ifsolved) {
    for (int i = 0; i<size; i++)
      free(m[i]);
    free(m);
    return false;
  }
  //   int* assignpoint = (int*)malCOMMENTloc(size*sizeof(int)); //malCOMMENTloc(estpts.size()*sizeof(int));
  for (int i = 0; i < size; i++) {
    int assign = -1;
    for (int j = 0; j < size; j++) {
      if (p.assignment[i][j] == 1) {
        assign = j;
        break;
      }
    }

    //            assignpoint[i] = assign;
    assingmentVec->push_back(assign);
  }

  //hungarian_print_assignment(&p);
  //hungarian_print_costmatrix(&p);


  //free(dist);


  for (int i = 0; i<size; i++)
    free(m[i]);

  free(m);

  return true;

}
void CHungarianMethod::AllocateCostMatrix(void)
{

  if (costMatrix != 0) {
    int cM = costMatrixSize;
    DeAllocateCostMatrix();
    costMatrixSize = cM;
  }



  int i, j;

  costMatrix = (int**)calloc(costMatrixSize, sizeof(int*));
  for (i = 0; i<costMatrixSize; i++) {
    costMatrix[i] = (int*)calloc(costMatrixSize, sizeof(int));
    for (j = 0; j<costMatrixSize; j++)
      costMatrix[i][j] = 0;
  }

}

void CHungarianMethod::DeAllocateCostMatrix(void)
{
  if (costMatrix == 0)
    return;

  for (int i = 0; i<costMatrixSize; i++)
    free(costMatrix[i]);

  free(costMatrix);

  costMatrixSize = 0;
  costMatrix = 0;
}

bool CHungarianMethod::SetCostData(int i, int j, int cost)
{
  if (costMatrixSize <= i || costMatrixSize <= j)
    return false;

  costMatrix[i][j] = cost;

  return true;
}

bool CHungarianMethod::CalcAssignment(std::vector<int> * assingmentVec)
{
  if (inited)
    hungarian_free(&p);


  int matrix_size = hungarian_init(&p, costMatrix, costMatrixSize, costMatrixSize, HUNGARIAN_MODE_MINIMIZE_COST);
  inited = true;

  //printf("Before solve\n");
  //hungarian_print_costmatrix(&p);


  bool ifsolved = hungarian_solve(&p);

  if (!ifsolved) {
    // @todo - use the logger from logger.h here. And also instead of printf.
    cout << "Hungarian match failed\n";
    return false;
  }
  //hungarian_solve(&p);

  //hungarian_print_assignment(&p);
  //   int* assignpoint = (int*)malCOMMENTloc(size*sizeof(int)); //malCOMMENTloc(estpts.size()*sizeof(int));
  for (int i = 0; i < costMatrixSize; i++) {
    int assign = -1;
    for (int j = 0; j < costMatrixSize; j++) {
      if (p.assignment[i][j] == 1) {
        assign = j;
        break;
      }
    }

    //            assignpoint[i] = assign;
    assingmentVec->push_back(assign);
  }

  return true;
}

void CHungarianMethod::InitCostMatrix(int cMSize)
{
  costMatrixSize = cMSize;
  AllocateCostMatrix();
}

