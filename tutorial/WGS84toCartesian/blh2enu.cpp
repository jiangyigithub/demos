/***********************************************************
  BLH -> ENU 変換
  : WGS84 の緯度(Beta)／経度(Lambda)／楕円体高(Height)を
    ENU (East/North/Up; 地平) 座標に変換する。
    * 途中、 ECEF（Earth Centered Earth Fixed; 地球中心・地
      球固定直交座標系）座標への変換を経由。

    DATE        AUTHOR       VERSION
    2021.05.06  mk-mode.com  1.00 新規作成

  Copyright(C) 2021 mk-mode.com All Rights Reserved.

  ----------------------------------------------------------
  引数 : B_0 L_0 H_0 B_1 L_1 H_1
         * B_0, L_0, H_0: 基準の BLH(WGS84) 座標
         * B_1, L_1, H_1: 対象の BLH(WGS84) 座標
  ----------------------------------------------------------
  $ g++102 -std=c++17 -Wall -O2 --pedantic-errors -o blh2enu blh2enu.cpp
***********************************************************/
#include <cmath>
#include <cstdlib>   // for EXIT_XXXX
#include <iomanip>
#include <iostream>
#include <vector>

namespace blh2enu {

// 定数
static constexpr double kPi    = atan(1.0) * 4.0;
static constexpr double kPi180 = kPi / 180;
// [ WGS84 座標パラメータ ]
// a(地球楕円体長半径(赤道面平均半径))
static constexpr double kA     = 6378137.0;
// 1 / f(地球楕円体扁平率=(a - b) / a)
static constexpr double k1F    = 298.257223563;
// b(地球楕円体短半径)
static constexpr double kB     = kA * (1.0 - 1.0 / k1F);
// e^2 = 2 * f - f * f
//     = (a^2 - b^2) / a^2
static constexpr double kE2    = (1.0 / k1F) * (2.0 - (1.0 / k1F));
// e'^2= (a^2 - b^2) / b^2
static constexpr double kEd2   = kE2 * kA * kA / (kB * kB);

// 座標
struct CoordB {
  double b;  // B(Beta)
  double l;  // L(Lambda)
  double h;  // H(Height)
};
struct CoordX {
  double x;  // X
  double y;  // Y
  double z;  // Z
};
struct CoordE {
  double e;  // E(East)
  double n;  // N(North)
  double u;  // U(Up)
};

/*
 * @brief      x 軸を軸とした回転行列
 *
 * @param[in]  回転量（°） (double)
 * @return     回転行列(3x3) (vector<vector<double>>)
 */
std::vector<std::vector<double>> mat_x(double ang) {
  double a;
  double c;
  double s;
  std::vector<std::vector<double>> mat(3, std::vector<double>(3, 0.0));

  try {
    a = ang * kPi180;
    c = cos(a);
    s = sin(a);
    mat[0][0] = 1.0;
    mat[1][1] =   c;
    mat[1][2] =   s;
    mat[2][1] =  -s;
    mat[2][2] =   c;
  } catch (...) {
    throw;
  }

  return mat;
}

/*
 * @brief      y 軸を軸とした回転行列
 *
 * @param[in]  回転量（°） (double)
 * @return     回転行列(3x3) (vector<vector<double>>)
 */
std::vector<std::vector<double>> mat_y(double ang) {
  double a;
  double c;
  double s;
  std::vector<std::vector<double>> mat(3, std::vector<double>(3, 0.0));

  try {
    a = ang * kPi180;
    c = cos(a);
    s = sin(a);
    mat[0][0] =   c;
    mat[0][2] =  -s;
    mat[1][1] = 1.0;
    mat[2][0] =   s;
    mat[2][2] =   c;
  } catch (...) {
    throw;
  }

  return mat;
}

/*
 * @brief      z 軸を軸とした回転行列
 *
 * @param[in]  回転量（°） (double)
 * @return     回転行列(3x3) (vector<vector<double>>)
 */
std::vector<std::vector<double>> mat_z(double ang) {
  double a;
  double c;
  double s;
  std::vector<std::vector<double>> mat(3, std::vector<double>(3, 0.0));

  try {
    a = ang * kPi180;
    c = cos(a);
    s = sin(a);
    mat[0][0] =   c;
    mat[0][1] =   s;
    mat[1][0] =  -s;
    mat[1][1] =   c;
    mat[2][2] = 1.0;
  } catch (...) {
    throw;
  }

  return mat;
}

/*
 * @brief      2行列(3x3)の積
 *
 * @param[in]  元の 3x3 行列 (vector<vector<double>>)
 * @param[in]  元の 3x3 行列 (vector<vector<double>>)
 * @return     計算後の 3x3 行列 (vector<vector<double>>)
 */
std::vector<std::vector<double>> mul_mat(
    std::vector<std::vector<double>> mat_a,
    std::vector<std::vector<double>> mat_b) {
  unsigned int i;
  unsigned int j;
  unsigned int k;
  std::vector<std::vector<double>> mat(3, std::vector<double>(3, 0.0));

  try {
    for (i = 0; i < 3; ++i) {
      for (j = 0; j < 3; ++j) {
        for (k = 0; k < 3; ++k) {
          mat[i][j] += mat_a[i][k] * mat_b[k][j];
        }
      }
    }
  } catch (...) {
    throw;
  }

  return mat;
}

/*
 * @brief      座標の回転
 *
 * @param[in]  3x3 回転行列 (vector<vector<double>>)
 * @param[in]  回転前座標 (CoordX)
 * @return     回転後座標 (CoordE)
 */
CoordE rotate(std::vector<std::vector<double>> mat_r, CoordX pt_0) {
  CoordE pt;

  try {
    pt.e = mat_r[0][0] * pt_0.x
         + mat_r[0][1] * pt_0.y
         + mat_r[0][2] * pt_0.z;
    pt.n = mat_r[1][0] * pt_0.x
         + mat_r[1][1] * pt_0.y
         + mat_r[1][2] * pt_0.z;
    pt.u = mat_r[2][0] * pt_0.x
         + mat_r[2][1] * pt_0.y
         + mat_r[2][2] * pt_0.z;
  } catch (...) {
    throw;
  }

  return pt;
}

/*
 * @brief      関数 N
 *
 * @param[in]  X (double)
 * @return     計算結果 (double)
 */
double n(double x) {
  double res;

  try {
    res = kA / sqrt(1.0 - kE2 * pow(sin(x * kPi180), 2));
  } catch (...) {
    throw;
  }

  return res;
}

/*
 * @brief      BLH -> ECEF
 *
 * @param[in]  BLH  座標 (CoordB)
 * @return     ECEF 座標 (CoordX)
 */
CoordX blh2ecef(CoordB c_src) {
  CoordX ecef;

  try {
    ecef.x = (n(c_src.b) + c_src.h)
            * cos(c_src.b * kPi180)
            * cos(c_src.l * kPi180);
    ecef.y = (n(c_src.b) + c_src.h)
            * cos(c_src.b * kPi180)
            * sin(c_src.l * kPi180);
    ecef.z = (n(c_src.b) * (1.0 - kE2) + c_src.h)
            * sin(c_src.b * kPi180);
  } catch (...) {
    throw;
  }

  return ecef;
}

/*
 * @brief      BLH -> ENU
 *
 * @param[in]  BLH 座標（基準） (CoordB)
 * @param[in]  BLH 座標（対象） (CoordB)
 * @return     ENU 座標 (CoordE)
 */
CoordE blh2enu(CoordB c_0, CoordB c_1) {
  CoordX ecef_x_0;
  CoordX ecef_x_1;
  CoordX ecef_x;
  std::vector<std::vector<double>> mat_0(3, std::vector<double>(3, 0.0));
  std::vector<std::vector<double>> mat_1(3, std::vector<double>(3, 0.0));
  std::vector<std::vector<double>> mat_2(3, std::vector<double>(3, 0.0));
  std::vector<std::vector<double>> mat;
  CoordE enu;

  try {
    // BLH -> ECEF
    ecef_x_0 = blh2ecef(c_0);
    ecef_x_1 = blh2ecef(c_1);
    ecef_x = {ecef_x_1.x - ecef_x_0.x,
              ecef_x_1.y - ecef_x_0.y,
              ecef_x_1.z - ecef_x_0.z};
    // 回転行列生成
    mat_0 = mat_z(90.0);
    mat_1 = mat_y(90.0 - c_0.b);
    mat_2 = mat_z(c_0.l);
    mat = mul_mat(mul_mat(mat_0, mat_1), mat_2);
    enu = rotate(mat, ecef_x);
  } catch (...) {
    throw;
  }

  return enu;
}

}  // namespace blh2enu

int main(int argc, char* argv[]) {
  namespace ns = blh2enu;
  ns::CoordB blh_0;  // 変換前の座標（基準）
  ns::CoordB blh_1;  // 変換前の座標（対象）
  ns::CoordE enu;    // 変換後の座標
  double az;         // 方位角
  double el;         // 仰角
  double dst;        // 距離

  try {
    // コマンドライン引数の個数チェック
    if (argc < 7) {
      std::cout << "Usage: ./blh2enu B_0 L_0 H_0 B_1 L_1 H_1" << std::endl;
      return EXIT_SUCCESS;
    }

    // 基準、対象の BLH(WGS84) 座標取得
    blh_0.b = std::stod(argv[1]);
    blh_0.l = std::stod(argv[2]);
    blh_0.h = std::stod(argv[3]);
    blh_1.b = std::stod(argv[4]);
    blh_1.l = std::stod(argv[5]);
    blh_1.h = std::stod(argv[6]);

    // BLH -> ENU
    enu = ns::blh2enu(blh_0, blh_1);

    // 方位角、仰角、距離
    az = atan2(enu.e, enu.n) / ns::kPi180;
    if (az < 0.0) { az += 360.0; }
    el = atan2(enu.u, sqrt(enu.e * enu.e + enu.n * enu.n)) / ns::kPi180;
    dst = 0;
    dst += enu.e * enu.e;
    dst += enu.n * enu.n;
    dst += enu.u * enu.u;
    dst = sqrt(dst);

    // 結果出力
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "BLH_0: LATITUDE(BETA) = "
              << std::setw(12) << blh_0.b << " °" << std::endl;
    std::cout << "    LONGITUDE(LAMBDA) = "
              << std::setw(12) << blh_0.l << " °" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "               HEIGHT = "
              << std::setw(7)  << blh_0.h << " m" << std::endl;
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "BLH:   LATITUDE(BETA) = "
              << std::setw(12) << blh_1.b << " °" << std::endl;
    std::cout << "    LONGITUDE(LAMBDA) = "
              << std::setw(12) << blh_1.l << " °" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "               HEIGHT = "
              << std::setw(7)  << blh_1.h << " m" << std::endl;
    std::cout << "-->" << std::endl;
    std::cout << "ENU: E = "
              << std::setw(12) << enu.e << " m" << std::endl;
    std::cout << "     N = "
              << std::setw(12) << enu.n << " m" << std::endl;
    std::cout << "     U = "
              << std::setw(12) << enu.u << " m" << std::endl;
    std::cout << "方位角 = "
              << std::setw(12) << az  << " °" << std::endl;
    std::cout << "  仰角 = "
              << std::setw(12) << el  << " °" << std::endl;
    std::cout << "  距離 = "
              << std::setw(12) << dst << " m" << std::endl;
  } catch (...) {
      std::cerr << "EXCEPTION!" << std::endl;
      return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
