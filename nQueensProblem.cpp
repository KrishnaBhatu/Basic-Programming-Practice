#include <cstdlib>
#include <iostream>
#include <stdio.h>

bool check_current_position(bool **problem_grid, int current_row,
                            int current_col, int n) {
  // check row
  int diagonal_right_start_row = abs(current_col - current_row);
  int diagonal_right_start_col = 0;
  int diagonal_left_start_col = n - 1;
  int diagonal_left_start_row = current_col + current_row - n + 1;
  int temp_row = 0;
  int temp_col = current_col + current_row;
  for (int i = 0; i < n; i++) {
    if (problem_grid[current_row][i])
      return false;
    if (problem_grid[i][current_col])
      return false;

    if (current_row + current_col < n) {
      if (i < current_row + current_col + 1)
        if (problem_grid[temp_row][temp_col])
          return false;
      temp_row++;
      temp_col--;
    } else {
      if (i < ((2 * n) - (current_col + current_row + 1))) {
        if (problem_grid[diagonal_left_start_row][diagonal_left_start_col])
          return false;
        diagonal_left_start_row++;
        diagonal_left_start_col--;
      }
    }

    if (i < (n - abs(current_col - current_row))) {
      if (current_row - current_col < 0) {
        if (problem_grid[diagonal_right_start_col][diagonal_right_start_row])
          return false;
        diagonal_right_start_row++;
        diagonal_right_start_col++;
      } else {
        if (problem_grid[diagonal_right_start_row][diagonal_right_start_col])
          return false;
        diagonal_right_start_row++;
        diagonal_right_start_col++;
      }
    }
  }
  return true;
  // check col
  // check diaog
}

bool possible_solution_generator(int n, bool **problem_grid,
                                 int current_position) {
  // If position is ture and there is the spot is safe then only return
  // For every location of that row
  // Execute till all the possible cols of 1 row are explored
  if (current_position == n)
    return true;

  for (int i = 0; i < n; i++) {
    // current location explored is [current_position][i]
    // if possible location the stay in loop or else iterate
    if (check_current_position(problem_grid, current_position, i, n)) {
      problem_grid[current_position][i] = true;
      if (possible_solution_generator(n, problem_grid, current_position + 1))
        return true;
      problem_grid[current_position][i] = false;
    }
  }
  return false;
}
void print_solution(int n, bool **problem_grid) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      std::cout << problem_grid[i][j] << ", ";
    }
    std::cout << std::endl;
  }
  return;
}

void caluclate_solution(int n) {
  bool **problem_grid;
  problem_grid = new bool *[n];
  for (int i = 0; i < n + 1; i++)
    problem_grid[i] = new bool[n];

  int current_position = 0;
  if (possible_solution_generator(n, problem_grid, current_position))
    print_solution(n, problem_grid);
  else
    std::cout << "no solution found  " << std::endl;

  return;
}

int main() {
  int problem_size = 0;
  std::cout << "Enter the size of problem: ";
  std::cin >> problem_size;
  std::cout << std::endl << "Calculating Solution ....." << std::endl;
  caluclate_solution(problem_size);
  return 0;
}