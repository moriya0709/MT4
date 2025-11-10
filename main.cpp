#include <Novice.h>
#include <cmath>
#include <algorithm>
#include <numbers>

const char kWindowTitle[] = "k024g0016";

struct Vector3 { float x, y, z; };
struct Matrix4x4 { float m[4][4]; };

struct Quaternion {
	float x;
	float y;
	float z;
	float w;
};


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
		} else if (std::fabs(f.y) < std::fabs(f.x) && std::fabs(f.y) < std::fabs(f.z)) {
			arbitrary = { 0.0f, 1.0f, 0.0f };
		} else {
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

// Quaternionの積
Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs) {
	Quaternion result;

	result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
	result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
	result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
	result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;

	return result;
}

// 単位Quaternionを返す
Quaternion IdentityQuaternion() {
	Quaternion result;
	result.x = 0.0f;
	result.y = 0.0f;
	result.z = 0.0f;
	result.w = 1.0f;
	return result;
}

// 共役Quaternionを返す
Quaternion Conjugate(const Quaternion& quaternion) {
	Quaternion result;
	result.x = -quaternion.x;
	result.y = -quaternion.y;
	result.z = -quaternion.z;
	result.w = quaternion.w;
	return result;
}

// Quaternionのnormを返す
float Norm(const Quaternion& quaternion) {
	return std::sqrt(
		quaternion.x * quaternion.x +
		quaternion.y * quaternion.y +
		quaternion.z * quaternion.z +
		quaternion.w * quaternion.w
	);
}

// 正規化したQuaternionを返す
Quaternion Normalize(const Quaternion& quaternion) {
	float norm = std::sqrt(
		quaternion.x * quaternion.x +
		quaternion.y * quaternion.y +
		quaternion.z * quaternion.z +
		quaternion.w * quaternion.w
	);

	Quaternion result;

	if (norm == 0.0f) {
		// 安全策：ゼロ除算を防ぐ
		result = { 0.0f, 0.0f, 0.0f, 1.0f };  // 単位クォータニオンを返す
	} else {
		float inv = 1.0f / norm;
		result.x = quaternion.x * inv;
		result.y = quaternion.y * inv;
		result.z = quaternion.z * inv;
		result.w = quaternion.w * inv;
	}

	return result;
}

// 逆Quaternionを返す
Quaternion Inverse(const Quaternion& quaternion) {
	float normSq =
		quaternion.x * quaternion.x +
		quaternion.y * quaternion.y +
		quaternion.z * quaternion.z +
		quaternion.w * quaternion.w;

	// ゼロ除算を防ぐ
	if (normSq == 0.0f) {
		return { 0.0f, 0.0f, 0.0f, 1.0f }; // 安全策：単位クォータニオンを返す
	}

	Quaternion conj = Conjugate(quaternion);
	float invNorm = 1.0f / normSq;

	return {
		conj.x * invNorm,
		conj.y * invNorm,
		conj.z * invNorm,
		conj.w * invNorm
	};
}


// スクリーン表示
static const int kColumnWidth = 60;
static const int kRowHeight = 20;
// 結果を描画
void MatrixScreenPrint(int x, int y, const Matrix4x4& matrix, const char* name) {
	for (int row = 2; row < 6; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(x + column * kColumnWidth, y + row * kRowHeight, "%6.03f", matrix.m[row - 2][column]);
		}
	}

	Novice::ScreenPrintf(x, y + kRowHeight, "%s", name);
}

// 3事件ベクトルの数値を表示する
void QuaternionScreenPrintf(int x, int y, const Quaternion& quaternion, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", quaternion.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", quaternion.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", quaternion.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%.02f", quaternion.w);
	Novice::ScreenPrintf(x + kColumnWidth * 4, y, "%s", label);
}

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	Quaternion q1 = { 2.0f,3.0f,4.0f,1.0f };
	Quaternion q2 = { 1.0f,3.0f,5.0f,2.0f };
	Quaternion identity = IdentityQuaternion();
	Quaternion conj = Conjugate(q1);
	Quaternion inv = Inverse(q1);
	Quaternion normal = Normalize(q1);
	Quaternion mul1 = Multiply(q1, q2);
	Quaternion mul2 = Multiply(q2, q1);
	float norm = Norm(q1);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

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

		QuaternionScreenPrintf(0, 0, identity, "Identity");
		QuaternionScreenPrintf(0, kRowHeight * 1, conj, "Conjugate");
		QuaternionScreenPrintf(0, kRowHeight * 2, inv, "Inverse");
		QuaternionScreenPrintf(0, kRowHeight * 3, normal, "Normalize");
		QuaternionScreenPrintf(0, kRowHeight * 4, mul1, "Multiply(q1,q2)");
		QuaternionScreenPrintf(0, kRowHeight * 5, mul2, "Multiply(q2,q1)");
		Novice::ScreenPrintf(0, kRowHeight * 6, "%.02f", norm);
		Novice::ScreenPrintf(kColumnWidth * 4, kRowHeight * 6, "Norm");

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
