// In this quiz you'll fit a polynomial to waypoints.

#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include<fstream> 
#include<string>
#include<sstream>
#include <vector> 

using namespace Eigen;
using namespace std;

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {

	// Read waypoint
	ifstream ifs("C:/Users/10001161181/Documents/GitHub/CarND-MPC-Quizzes/polyfit/src/lake_track_waypoints.csv");

	if (!ifs) {
		cout << "入力エラー";
		return 1;
	}


	//csvファイルを1行ずつ読み込む
	string str;
	int cnt = 0;
	std::vector<double> x_vec, y_vec;

	while (getline(ifs, str)) {
		string token;
		istringstream stream(str);
		int cnt_2 = 0;

		//1行のうち、文字列とコンマを分割する
		while (getline(stream, token, ',')) {
			//すべて文字列として読み込まれるため
			//数値は変換が必要
			if (cnt > 0) {
				double temp = stof(token); //stof(string str) : stringをfloatに変換
				if (cnt_2 == 0) {
					//cout << temp << ",";
					x_vec.push_back(temp);
				}
				else{
					//cout << temp << ",";
					y_vec.push_back(temp);
				}
				cnt_2 += 1;
			}
			
		}
		cnt += 1;
		cout << endl;
	}

  Eigen::VectorXd xvals(x_vec.size());
  Eigen::VectorXd yvals(y_vec.size());
  for (int i = 0; i < x_vec.size(); i++) {
	  xvals[i] = x_vec[i];
	  yvals[i] = y_vec[i];
  }

  // x waypoint coordinates
  //xvals << 9.261977, -2.06803, -19.6663, -36.868, -51.6263, -66.3482;
  // y waypoint coordinates
  //yvals << 5.17, -2.25, -15.306, -29.46, -42.85, -57.6116;

  // Pass the x and y waypoint coordinates along the order of the polynomial.
  // In this case, 3.
  auto coeffs = polyfit(xvals, yvals, 3);

  for (double x = 0; x <= 20; x += 1.0) {
    // We can evaluate the polynomial at a x coordinate by calling `polyeval`.
    // The first argument being the coefficients, `coeffs`.
    // The second being the x coordinate, `x`.
    auto v = polyeval(coeffs, x);
    std::cout << v << std::endl;
  }

  // Expected output
  // -0.905562
  // -0.226606
  // 0.447594
  // 1.11706
  // 1.7818
  // 2.44185
  // 3.09723
  // 3.74794
  // 4.39402
  // 5.03548
  // 5.67235
  // 6.30463
  // 6.93236
  // 7.55555
  // 8.17423
  // 8.7884
  // 9.3981
  // 10.0033
  // 10.6041
  // 11.2005
  // 11.7925
}