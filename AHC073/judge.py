# AHC073 A 채점기: python3 judge.py <input> <output>  -> 점수 출력 (규칙 위반 시 예외)
import sys

def judge(inp, out):
    it = inp.split()
    N, M, K, T = map(int, it[:4])
    s = (int(it[4]), int(it[5]))
    g = [(int(it[6 + 2 * i]), int(it[7 + 2 * i])) for i in range(M)]
    lines = out.strip().split("\n")
    circles = {}
    for k in range(K):
        a, b = map(int, lines[k].split())
        assert 0 <= a < N and 0 <= b < N, "circle out of grid"
        circles[(a, b)] = circles.get((a, b), 0) + 1
    ops = lines[K:]
    assert len(ops) <= T, f"too many ops {len(ops)} > {T}"
    pos, V = s, 0
    D = {"U": (-1, 0), "D": (1, 0), "L": (0, -1), "R": (0, 1)}
    for op in ops:
        p = op.split()
        if p[0] in D:
            ni, nj = pos[0] + D[p[0]][0], pos[1] + D[p[0]][1]
            assert 0 <= ni < N and 0 <= nj < N, "walk out of grid"
            pos = (ni, nj)
        else:
            assert p[0] == "J" and len(p) == 3, f"bad op {op}"
            a, b = int(p[1]), int(p[2])
            assert circles.get((a, b), 0) > 0, f"no circle at {(a, b)}"
            ni, nj = 2 * a - pos[0], 2 * b - pos[1]
            assert 0 <= ni < N and 0 <= nj < N, "teleport out of grid"
            circles[(a, b)] -= 1
            circles[pos] = circles.get(pos, 0) + 1
            pos = (ni, nj)
        if V < M and pos == g[V]:
            V += 1
    A = len(ops)
    return (M + T - A + 1, A, T) if V == M else (V + 1, A, T)

if __name__ == "__main__":
    print(*judge(open(sys.argv[1]).read(), open(sys.argv[2]).read()))
