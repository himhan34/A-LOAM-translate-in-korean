// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once  // 헤더 파일이 중복 포함되는 것을 방지

#include <ctime>  // 시간 관련 함수를 제공하는 헤더 파일
#include <cstdlib>  // 표준 라이브러리의 다양한 유틸리티 함수 포함
#include <chrono>  // 시간 측정과 관련된 라이브러리 포함

class TicToc  // TicToc 클래스를 정의
{
  public:
    TicToc()  // 생성자: 객체가 생성될 때 tic() 함수를 호출하여 시작 시간을 기록
    {
        tic();
    }

    void tic()  // tic 함수: 현재 시간을 시작 시간으로 기록
    {
        start = std::chrono::system_clock::now();
    }

    double toc()  // toc 함수: 시작 시간부터 현재 시간까지의 경과 시간을 밀리초 단위로 반환
    {
        end = std::chrono::system_clock::now();  // 현재 시간을 종료 시간으로 기록
        std::chrono::duration<double> elapsed_seconds = end - start;  // 경과 시간을 초 단위로 계산
        return elapsed_seconds.count() * 1000;  // 밀리초 단위로 변환하여 반환
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;  // 시작 시간과 종료 시간을 저장할 변수
};
