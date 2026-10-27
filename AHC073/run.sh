#!/bin/bash
# 사용법: ./run.sh <솔버 실행파일> <케이스 수>
#   입력은 in/0000.txt ... (python3 gen.py 0 150 in 으로 생성), 출력은 out/ 에 저장
#   출력: 평균 조작 수 A, 평균 점수, 최대 실행 시간(ms)
BIN=$1; CNT=$2; shift 2
mkdir -p out
tot=0; totA=0; maxt=0
for i in $(seq 0 $((CNT-1))); do
  f=$(printf "in/%04d.txt" $i)
  o=$(printf "out/%04d.txt" $i)
  s=$(date +%s%N)
  ./$BIN "$@" < $f > $o 2>/dev/null
  e=$(( ($(date +%s%N) - s) / 1000000 ))
  [ $e -gt $maxt ] && maxt=$e
  read sc A T < <(python3 judge.py $f $o)
  tot=$((tot+sc)); totA=$((totA+A))
done
echo "avgA=$((totA/CNT)) avgScore=$((tot/CNT)) maxMs=$maxt"
