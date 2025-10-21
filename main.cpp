#include <Novice.h>
#include <cmath>
#include <algorithm>
#include <numbers>

const char kWindowTitle[] = "k024g0016";

struct Vector3 { float x, y, z; };
struct Matrix4x4 { float m[4][4]; };

static Vector3 Normalize(const Vector3& v) {
    float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len == 0.0f) return { 0.0f, 0.0f, 0.0f };
    return { v.x / len, v.y / len, v.z / len };
}

static float Dot(const Vector3& a, const Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vector3 Cross(const Vector3& a, const Vector3& b) {
    return { a.y * b.z - a.z * b.y,
             a.z * b.x - a.x * b.z,
             a.x * b.y - a.y * b.x };
}

static Matrix4x4 MakeIdentityMatrix() {
    Matrix4x4 I = {};
    I.m[0][0] = 1; I.m[1][1] = 1; I.m[2][2] = 1; I.m[3][3] = 1;
    return I;
}


Matrix4x4 MakeRotateAxisAngle(const Vector3& axis, float angle) {
	// 正規化
	Vector3 n = Normalize(axis);

	float x = n.x;
	float y = n.y;
	float z = n.z;
	float c = std::cos(angle);
	float s = std::sin(angle);
	float t = 1.0f - c;

	Matrix4x4 result = {};

	result.m[0][0] = t * x * x + c;
	result.m[0][1] = t * x * y + s * z;
	result.m[0][2] = t * x * z - s * y;
	result.m[0][3] = 0.0f;

	result.m[1][0] = t * x * y - s * z;
	result.m[1][1] = t * y * y + c;
	result.m[1][2] = t * y * z + s * x;
	result.m[1][3] = 0.0f;

	result.m[2][0] = t * x * z + s * y;
	result.m[2][1] = t * y * z - s * x;
	result.m[2][2] = t * z * z + c;
	result.m[2][3] = 0.0f;

	result.m[3][0] = 0.0f;
	result.m[3][1] = 0.0f;
	result.m[3][2] = 0.0f;
	result.m[3][3] = 1.0f;

	return result;
}

Matrix4x4 DirectionToDirection(const Vector3& from, const Vector3& to) {
    const float kEps = 1e-6f;
    float lenFrom = std::sqrt(from.x * from.x + from.y * from.y + from.z * from.z);
    float lenTo = std::sqrt(to.x * to.x + to.y * to.y + to.z * to.z);
    if (lenFrom < kEps || lenTo < kEps) {
        return MakeIdentityMatrix();
    }

    Vector3 f = { from.x / lenFrom, from.y / lenFrom, from.z / lenFrom };
    Vector3 t = { to.x / lenTo,     to.y / lenTo,     to.z / lenTo };
    float dot = std::clamp(Dot(f, t), -1.0f, 1.0f);

    if (dot > 1.0f - 1e-6f) {
        return MakeIdentityMatrix();
    }

    // 180度に近い場合：決定論的に最小成分方向を使って直交軸を選ぶ
    if (dot < -1.0f + 1e-6f) {
        Vector3 arbitrary;
        if (std::fabs(f.x) < std::fabs(f.y) && std::fabs(f.x) < std::fabs(f.z)) {
            arbitrary = { 1.0f, 0.0f, 0.0f };
        }
		else if (std::fabs(f.y) < std::fabs(f.x) && std::fabs(f.y) < std::fabs(f.z)) {
            arbitrary = { 0.0f, 1.0f, 0.0f };
        }
		else {
            arbitrary = { 0.0f, 0.0f, 1.0f };
        }
        Vector3 axis = Cross(f, arbitrary);
        float axisLen = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
        if (axisLen < kEps) {
            axis = { -f.y, f.x, 0.0f };
            axisLen = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
            if (axisLen < kEps) return MakeIdentityMatrix();
        }
        axis = { axis.x / axisLen, axis.y / axisLen, axis.z / axisLen };
        return MakeRotateAxisAngle(axis, std::numbers::pi_v<float>);
    }

    Vector3 axis = Cross(f, t);
    float axisLen = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (axisLen < kEps) return MakeIdentityMatrix();
    axis = { axis.x / axisLen, axis.y / axisLen, axis.z / axisLen };
    float angle = std::acos(dot);
    return MakeRotateAxisAngle(axis, angle);
}

// スクリーン表示
static const int kColumnWidth = 60;
static const int kRowHeight = 20;

void MatrixScreenPrint(int x, int y, const Matrix4x4& matrix, const char* name) {
	for (int row = 2; row < 6; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(x + column * kColumnWidth, y + row * kRowHeight, "%6.03f", matrix.m[row - 2][column]);
		}
	}

	Novice::ScreenPrintf(x, y + kRowHeight, "%s", name);
}


int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	Vector3 from0 = Normalize(Vector3{ 1.0f,0.7f,0.5f });
	Vector3 to0;
	to0.x = -from0.x;
	to0.y = -from0.y;
	to0.z = -from0.z;
	Vector3 from1 = Normalize(Vector3{ -0.6f,0.9f,0.2f });
	Vector3 to1 = Normalize(Vector3{ 0.4f,0.7f,-0.5f });
	
	Matrix4x4 rotateMatrix0 = DirectionToDirection(Normalize(Vector3{1.0f,0.0f,0.0f}), Normalize(Vector3{ -1.0f,0.0f,0.0f }));
	Matrix4x4 rotateMatrix1 = DirectionToDirection(from0, to0);
	Matrix4x4 rotateMatrix2 = DirectionToDirection(from1, to1);
	

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		MatrixScreenPrint(0, 0, rotateMatrix0, "rotateMatrix0");
		MatrixScreenPrint(0, kRowHeight * 5, rotateMatrix1, "rotateMatrix1");
		MatrixScreenPrint(0, kRowHeight * 10, rotateMatrix2, "rotateMatrix2");

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
