#ifndef EGEOMETRY_VECTOR3_H
#define EGEOMETRY_VECTOR3_H

#include <assert.h>
#include <math.h>
#include <iostream>

namespace egeometry
{

/// float-vector にほぼ対応するクラス
class Vector3
{
public:
    /// 引数なしコンストラクタ。すべての要素を0で初期化
    inline Vector3()
    {
        data_[0] = 0.0;
        data_[1] = 0.0;
        data_[2] = 0.0;
        data_[3] = 0.0;
    };

    /// 引数ありコンストラクタ。すべての要素を指定
    inline Vector3(const double x, const double y, const double z)
    {
        data_[0] = x;
        data_[1] = y;
        data_[2] = z;
        data_[3] = 0.0;
    };

    /// クラス用コピーコンストラクタ。なんでも受け取り、要素をコピーする。
    template <class T>
    inline Vector3(const T& from)
    {
        data_[0] = from[0];
        data_[1] = from[1];
        data_[2] = from[2];
    };

    /// 配列用コピーコンストラクタ。なんでも受け取り、要素をコピーする。
    template <class T>
    inline Vector3(const T* from)
    {
        data_[0] = from[0];
        data_[1] = from[1];
        data_[2] = from[2];
    };

    /// クラス用代入。なんでも受け取る
    template <class T>
    inline Vector3& operator=(const T& from)
    {
        data_[0] = from[0];
        data_[1] = from[1];
        data_[2] = from[2];
        return *this;
    };

    /// 配列用代入。なんでも受け取る
    template <class T>
    inline Vector3& operator=(const T* from)
    {
        data_[0] = from[0];
        data_[1] = from[1];
        data_[2] = from[2];
        return *this;
    };

    /// 自分に加える。
    inline Vector3& operator+=(const Vector3& v)
    {
        data_[0] += v.data_[0];
        data_[1] += v.data_[1];
        data_[2] += v.data_[2];
        return *this;
    };

    /// 自分から引く
    inline Vector3& operator-=(const Vector3& v)
    {
        data_[0] -= v.data_[0];
        data_[1] -= v.data_[1];
        data_[2] -= v.data_[2];
        return *this;
    };

    /// スカラー倍する
    inline Vector3& operator*=(const double scale)
    {
        data_[0] *= scale;
        data_[1] *= scale;
        data_[2] *= scale;
        return *this;
    };

    /// スカラーで割る
    inline Vector3& operator/=(const double scale)
    {
        assert(scale != 0.0);
        data_[0] /= scale;
        data_[1] /= scale;
        data_[2] /= scale;
        return *this;
    };

    /// ベクトルの内積
    inline double Dot(const Vector3& v) const
    {
        return (data_[0] * v.data_[0] +
                data_[1] * v.data_[1] +
                data_[2] * v.data_[2]);
    };

    /// 絶対値の２乗を取得
    inline double Length2() const
    {
        return Dot(*this);
    };

    /// 絶対値を取得
    inline double Length() const
    {
        return sqrt(Length2());
    };
    
    /// 距離の２乗を取得
    /// @param [in] v 距離を計算する相手
    inline double Distance2(const Vector3& v) const;

    /// 距離を取得
    /// @param [in] v 距離を計算する相手
    inline double Distance(const Vector3& v) const;

    /// 正規化(サイズを１に)する
    inline Vector3& Normalize()
    {
        return *this /= Length();
    };

    /// 0番目の要素をコピー
    inline double GetX() const
    {
        return data_[0];
    }

    /// 1番目の要素をコピー
    inline double GetY() const
    {
        return data_[1];
    }
    
    /// 2番目の要素をコピー
    inline double GetZ() const
    {
        return data_[2];
    }

    /// 0番目の要素を設定
    /// @param [in] x 設定する値
    inline void SetX(double x)
    {
        data_[0] = x;
    };

    /// 1番目の要素を設定
    /// @param [in] y 設定する値
    inline void SetY(double y)
    {
        data_[1] = y;
    };

    /// 2番目の要素を設定
    /// @param [in] z 設定する値
    inline void SetZ(double z)
    {
        data_[2] = z;
    };

    /// 等しいかどうかの判定。すべての要素で==のとき。(浮動小数なので危険か)
    /// @param [in] other 比較対象
	inline bool operator==(const Vector3& other) const
	{
		return ((data_[2]==other.data_[2]) && (data_[1]==other.data_[1]) && (data_[0]==other.data_[0]));
	};
    
    /// 等しくないかの判定。
    /// @param [in] other 比較対象
	inline bool operator!=(const Vector3& other) const
	{
		return !(*this == other);
	};

    /// 要素へのアクセス
    /// @param [in] i 要素インデックス
    inline double& operator[](int i)
    { 
        assert(0 <= i && i < 3);
        return data_[i]; 
    }

    /// 要素へのアクセス(const版)
    /// @param [in] i 要素インデックス
    inline double operator[](int i) const
    { 
        assert(0 <= i && i < 3);
        return data_[i]; 
    }

    /// 値を一度に設定
    /// @param [in] x 0番目要素
    /// @param [in] y 1番目要素
    /// @param [in] z 2番目要素
    inline void SetValue(const double x, const double y, const double z)
    {
        data_[0]=x;
        data_[1]=y;
        data_[2]=z;
        data_[3] = 0.0;
    }
    
    /// 要素をすべて0にする
    inline void SetZero()
    {
        SetValue(0.0, 0.0, 0.0);
    }
    /// データ。
    double data_[4];
};

/// 引き算。各要素を引いたあたらしいVector3を返す
/// @param [in] v1 左辺
/// @param [in] v2 右辺
inline Vector3 operator-(const Vector3& v1, const Vector3& v2)
{
    return Vector3(v1.data_[0] - v2.data_[0], v1.data_[1] - v2.data_[1], v1.data_[2] - v2.data_[2]);
}

/// 符号反転。すべての要素の符号が反転する。
/// @param [in] v 反転するVector3
inline Vector3 operator-(const Vector3& v)
{
    return Vector3(-v.data_[0], -v.data_[1], -v.data_[2]);
}


inline double Vector3::Distance(const Vector3& v) const
{
    return (v - *this).Length();
}

inline double Vector3::Distance2(const Vector3& v) const
{
    return (v - *this).Length2();
}

/// 足し算。すべての要素を加える
/// @param [in] v1 左辺
/// @param [in] v2 右辺
inline Vector3 operator+(const Vector3& v1, const Vector3& v2) 
{
    return Vector3(v1.data_[0] + v2.data_[0], v1.data_[1] + v2.data_[1], v1.data_[2] + v2.data_[2]);
}

/// 掛け算。要素同士を掛け合わせたVector3を返す
/// @param [in] v1 左辺
/// @param [in] v2 右辺
inline Vector3 operator*(const Vector3& v1, const Vector3& v2) 
{
    return Vector3(v1.data_[0] * v2.data_[0], v1.data_[1] * v2.data_[1], v1.data_[2] * v2.data_[2]);
}

/// スカラー倍
/// @param [in] v 倍化されるベクトル
/// @param [in] s スカラー
inline Vector3 operator*(const Vector3& v, const double s)
{
    return Vector3(v.data_[0] * s, v.data_[1] * s, v.data_[2] * s);
}

/// スカラー倍
/// @param [in] s スカラー
/// @param [in] v 倍化されるベクトル
inline Vector3 operator*(const double s, const Vector3& v)
{ 
    return v * s; 
}

/// 1/スカラー倍
/// @param [in] v 倍化されるベクトル
/// @param [in] s スカラー
inline Vector3 operator/(const Vector3& v, const double s)
{
    assert(s != 0.0);
    return v * (1.0 / s);
}

/// @brief \#f(x y z)フォーマットで出力する
/// @param [in] stream 出力先ストリーム
/// @param [in] v 出力するベクトル
inline std::ostream& operator<<(std::ostream& stream, const Vector3& v)
{
    stream << "#f(" << v[0] << " " << v[1] << " " << v[2] << ")";
    return stream;
}


}

#endif //EUSGEOMETRY_VECTOR3_H
