# AHC073 A 입력 생성기 (문제의 Input Generation 절을 그대로 구현)
# 사용법: python3 gen.py <seed 시작> <개수> <출력 디렉터리>
import random, sys, os

N, M, K = 128, 150, 3

def gen(seed):
    rng = random.Random(seed)
    cells = rng.sample(range(N * N), M + 1)
    pts = [(c // N, c % N) for c in cells]
    s, g = pts[0], pts[1:]
    T, cur = 0, s
    for p in g:
        T += abs(p[0] - cur[0]) + abs(p[1] - cur[1])
        cur = p
    lines = [f"{N} {M} {K} {T}", f"{s[0]} {s[1]}"] + [f"{a} {b}" for a, b in g]
    return "\n".join(lines) + "\n"

if __name__ == "__main__":
    start, cnt, out = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
    os.makedirs(out, exist_ok=True)
    for sd in range(start, start + cnt):
        with open(os.path.join(out, f"{sd:04d}.txt"), "w") as f:
            f.write(gen(sd))
