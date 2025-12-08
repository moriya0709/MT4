#include <Novice.h>
#include <cmath>
#include <algorithm>
#include <numbers>
#include <cassert>

const char kWindowTitle[] = "k024g0016";

struct Vector3 { float x, y, z; };
struct Matrix4x4 { float m[4][4]; };

struct Quaternion {
	float x;
	float y;
	float z;
	float w;

	// クォータニオンの共役
	Quaternion Conjugate() const {
		return { -x, -y, -z, w };
	}

	// クォータニオン同士の掛け算
	Quaternion operator*(const Quaternion& q) const {
		return {
			w * q.x + x * q.w + y * q.z - z * q.y,
			w * q.y - x * q.z + y * q.w + z * q.x,
			w * q.z + x * q.y - y * q.x + z * q.w,
			w * q.w - x * q.x - y * q.y - z * q.z
		};
	}

	// 単項マイナス演算子
	Quaternion operator-=(const Quaternion& q) const {
		return { -q.x, -q.y, -q.z, -q.w };
	}
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

// 任意軸回転を表すQuaternionの生成
Quaternion MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle) {
	Vector3 nAxis = Normalize(axis);
	float halfAngle = angle * 0.5f;
	float s = std::sin(halfAngle);

	Quaternion q;
	q.x = nAxis.x * s;
	q.y = nAxis.y * s;
	q.z = nAxis.z * s;
	q.w = std::cos(halfAngle);
	return q;
}

// ベクトルをQuaternionで回転させた結果のベクトルを求める
Vector3 RotateVector(const Vector3& vector, const Quaternion& quaternion) {
	// ベクトルを w=0 のクォータニオンに変換
	Quaternion p = { vector.x, vector.y, vector.z, 0.0f };

	// 回転計算
	Quaternion rotated = quaternion * p * quaternion.Conjugate();

	// 結果をベクトルに戻す
	return { rotated.x, rotated.y, rotated.z };
}

// Quaternionから回転行列を求める
Matrix4x4 MakeRotateMatrix(const Quaternion& quaternion) {
	float xx = quaternion.x * quaternion.x;
	float yy = quaternion.y * quaternion.y;
	float zz = quaternion.z * quaternion.z;
	float xy = quaternion.x * quaternion.y;
	float xz = quaternion.x * quaternion.z;
	float yz = quaternion.y * quaternion.z;
	float wx = quaternion.w * quaternion.x;
	float wy = quaternion.w * quaternion.y;
	float wz = quaternion.w * quaternion.z;

	Matrix4x4 mat = {};

	mat.m[0][0] = 1.0f - 2.0f * (yy + zz);
	mat.m[0][1] = 2.0f * (xy + wz);
	mat.m[0][2] = 2.0f * (xz - wy);
	mat.m[0][3] = 0.0f;

	mat.m[1][0] = 2.0f * (xy - wz);
	mat.m[1][1] = 1.0f - 2.0f * (xx + zz);
	mat.m[1][2] = 2.0f * (yz + wx);
	mat.m[1][3] = 0.0f;

	mat.m[2][0] = 2.0f * (xz + wy);
	mat.m[2][1] = 2.0f * (yz - wx);
	mat.m[2][2] = 1.0f - 2.0f * (xx + yy);
	mat.m[2][3] = 0.0f;

	mat.m[3][0] = 0.0f;
	mat.m[3][1] = 0.0f;
	mat.m[3][2] = 0.0f;
	mat.m[3][3] = 1.0f;

	return mat;
}

// 座標変換
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	Vector3 result;
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + matrix.m[3][3];
	assert(w != 0.0f);
	result.x = (vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0]) / w;
	result.y = (vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1]) / w;
	result.z = (vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2]) / w;
	return result;
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

void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", vector.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", vector.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", vector.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%s", label);

}

// 球面線形補間
Quaternion Slerp(const Quaternion& q0, const Quaternion& q1, float t) {
	float dot = q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w;

	Quaternion q1Copy = q1;

	// 最短補間
	if (dot < 0.0f) {
		dot = -dot;
		q1Copy = { -q1.x, -q1.y, -q1.z, -q1.w };
	}

	const float epsilon = 1e-6f;

	// 角度が小さい場合は Lerp で近似
	if (1.0f - dot < epsilon) {
		Quaternion result = {
			q0.x + t * (q1Copy.x - q0.x),
			q0.y + t * (q1Copy.y - q0.y),
			q0.z + t * (q1Copy.z - q0.z),
			q0.w + t * (q1Copy.w - q0.w)
		};
		return Normalize(result);
	}

	float theta = std::acos(dot);
	float sinTheta = std::sin(theta);

	float w0 = std::sin((1.0f - t) * theta) / sinTheta;
	float w1 = std::sin(t * theta) / sinTheta;

	Quaternion result = {
		w0 * q0.x + w1 * q1Copy.x,
		w0 * q0.y + w1 * q1Copy.y,
		w0 * q0.z + w1 * q1Copy.z,
		w0 * q0.w + w1 * q1Copy.w
	};

	// ※コレが重要！
	return Normalize(result);
}


int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	Quaternion rotation0 = MakeRotateAxisAngleQuaternion({ 0.71f,0.71f,0.0f }, 0.3f);
	Quaternion rotation1 = MakeRotateAxisAngleQuaternion({ 0.71f,0.0f,0.71f }, 3.141592f);

	Quaternion interpolate0 = Slerp(rotation0, rotation1, 0.0f);
	Quaternion interpolate1 = Slerp(rotation0, rotation1, 0.3f);
	Quaternion interpolate2 = Slerp(rotation0, rotation1, 0.5f);
	Quaternion interpolate3 = Slerp(rotation0, rotation1, 0.7f);
	Quaternion interpolate4 = Slerp(rotation0, rotation1, 1.0f);

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

		QuaternionScreenPrintf(0, kRowHeight * 0, interpolate0, " : interpolate0,Slerp(q0,q1,0.0f)");
		QuaternionScreenPrintf(0, kRowHeight * 1, interpolate1, " : interpolate1,Slerp(q0,q1,0.3f)");
		QuaternionScreenPrintf(0, kRowHeight * 2, interpolate2, " : interpolate2,Slerp(q0,q1,0.5f)");
		QuaternionScreenPrintf(0, kRowHeight * 3, interpolate3, " : interpolate3,Slerp(q0,q1,0.7f)");
		QuaternionScreenPrintf(0, kRowHeight * 4, interpolate4, " : interpolate4,Slerp(q0,q1,1.0f)");
		
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
