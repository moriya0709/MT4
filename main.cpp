#include <Novice.h>
#include <cmath>

const char kWindowTitle[] = "k024g0016";

struct Vector3 {
	float x;
	float y;
	float z;
};

struct Matrix4x4 {
	float m[4][4];
};

Vector3 Normalize(const Vector3& v) {
	float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

	// ゼロ割防止
	if (length == 0.0f) {
		return { 0.0f, 0.0f, 0.0f };
	}

	return {
		v.x / length,
		v.y / length,
		v.z / length
	};
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

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	Vector3 axis = Normalize({ 1.0f,1.0f,1.0f });
	float angle = 0.44f;
	Matrix4x4 rotateMatrix = MakeRotateAxisAngle(axis, angle);

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

		MatrixScreenPrint(0, 0, rotateMatrix, "rotateMatrix");

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
