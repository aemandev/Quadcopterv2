#include <iostream>
#include "MyClass.hpp"
#include "MyClass.cpp"

int main() {
  // Default Initialization
  MyClass obj1;
  std::cout << "obj1.value: " << obj1.value << std::endl;

  return 0;
}