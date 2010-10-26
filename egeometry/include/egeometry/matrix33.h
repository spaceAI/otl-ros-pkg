#ifndef EGEOMETRY_MATRIX33_H
#define EGEOMETRY_MATRIX33_H

#include <egeometry/vector3.h>
#include <iostream>



namespace egeometry
{

/// 3x3の行列。回転行列に用いることを想定している。
/// コードはbtMatrix3x3を非常に参考にしています。というよりコピーに近い。
class Matrix33
{
public:
    /// デフォルトコンストラクタ。すべての要素は0
    inline Matrix33()
    {
        SetValue(0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0);
    };

    /// 要素指定コンストラクタ
    /// @param [in] xx 要素[0][0]
    /// @param [in] xy 要素[0][1]
    /// @param [in] xz 要素[0][2]
    /// @param [in] yx 要素[1][0]
    /// @param [in] yy 要素[1][1]
    /// @param [in] yz 要素[1][2]
    /// @param [in] zx 要素[2][0]
    /// @param [in] zy 要素[2][1]
    /// @param [in] zz 要素[2][2]
    inline Matrix33(const double xx, const double xy, const double xz,
                    const double yx, const double yy, const double yz,
                    const double zx, const double zy, const double zz)
    {
        SetValue(xx, xy, xz,
                 yx, yy, yz,
                 zx, zy, zz);
    };

    /// コピーコンストラクタ。値がコピーされる。
    /// @param [in] other コピー元Matrix
    inline Matrix33 (const Matrix33& other)
    {
        row_[0] = other.row_[0];
        row_[1] = other.row_[1];
        row_[2] = other.row_[2];
    }

    /// 代入演算子。値がコピーされる。
    /// @param [in] other コピー元Matrix
    inline Matrix33& operator=(const Matrix33& other)
    {
        row_[0] = other.row_[0];
        row_[1] = other.row_[1];
        row_[2] = other.row_[2];
        return *this;
    }

    /// 縦ベクトルを取得する
    /// @param [in] i 列番号
    inline Vector3 GetColumn(int i) const
    {
        assert(0 <= i && i < 3);
        return Vector3(row_[0][i],row_[1][i],row_[2][i]);
    }

    /// 横ベクトルを取得する
    /// @param [in] i 行番号
    inline Vector3 GetRow(int i) const
    {
        assert(0 <= i && i < 3);
        return row_[i];
    }

    /// 横ベクトルを取得する
    /// @param [in] i 行番号
    inline Vector3& operator[](int i)
    { 
        assert(0 <= i && i < 3);
        return row_[i]; 
    }

    /// 横ベクトルを取得する。const用
    /// @param [in] i 行番号
    inline const Vector3& operator[](int i) const
    { 
        assert(0 <= i && i < 3);
        return row_[i]; 
    }

    /// 行列を右からかける
    /// @param [in] m かける行列
    inline Matrix33& operator*=(const Matrix33& m)
    {
		SetValue(m.tdotx(row_[0]), m.tdoty(row_[0]), m.tdotz(row_[0]),
				 m.tdotx(row_[1]), m.tdoty(row_[1]), m.tdotz(row_[1]),
				 m.tdotx(row_[2]), m.tdoty(row_[2]), m.tdotz(row_[2]));
		return *this;
    };

    /// 0列との内積を計算する
    /// @param [in] v かけるベクトル
    inline double tdotx(const Vector3& v) const 
    {
        return row_[0].GetX() * v.GetX() + row_[1].GetX() * v.GetY() + row_[2].GetX() * v.GetZ();
    }

    /// 1列との内積を計算する
    /// @param [in] v かけるベクトル
    inline double tdoty(const Vector3& v) const 
    {
        return row_[0].GetY() * v.GetX() + row_[1].GetY() * v.GetY() + row_[2].GetY() * v.GetZ();
    }

    /// 2列との内積を計算する
    /// @param [in] v かけるベクトル
    inline double tdotz(const Vector3& v) const 
    {
        return row_[0].GetZ() * v.GetX() + row_[1].GetZ() * v.GetY() + row_[2].GetZ() * v.GetZ();
    }

    /// 逆行列を返す
	inline Matrix33 Inverse() const
	{
		Vector3 co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
		double det = (*this)[0].Dot(co);
		assert(det != 0.0);
		double s = 1.0 / det;
		return Matrix33(co.GetX() * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s,
                        co.GetY() * s, cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s,
                        co.GetZ() * s, cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) * s);
	}
	
    /// 値を一度に設定する
    /// @param [in] xx 要素[0][0]
    /// @param [in] xy 要素[0][1]
    /// @param [in] xz 要素[0][2]
    /// @param [in] yx 要素[1][0]
    /// @param [in] yy 要素[1][1]
    /// @param [in] yz 要素[1][2]
    /// @param [in] zx 要素[2][0]
    /// @param [in] zy 要素[2][1]
    /// @param [in] zz 要素[2][2]
    inline void SetValue(const double xx, const double xy, const double xz, 
                         const double yx, const double yy, const double yz, 
                         const double zx, const double zy, const double zz)
    {
        row_[0].SetValue(xx,xy,xz);
        row_[1].SetValue(yx,yy,yz);
        row_[2].SetValue(zx,zy,zz);
    }

    /// 単位行列を返す
    inline static const Matrix33& GetIdentity()
    {
        static const Matrix33 identityMatrix((1.0), (0.0), (0.0), 
                                                (0.0), (1.0), (0.0), 
                                                (0.0), (0.0), (1.0));
        return identityMatrix;
    }

    /// すべての要素を0にする
    inline void SetZero()
    {
        SetValue(0.0,0.0,0.0,
                 0.0,0.0,0.0,
                 0.0,0.0,0.0);
    }

    /// 転置行列を返す
	inline Matrix33 Transpose() const 
	{
		return Matrix33(row_[0].GetX(), row_[1].GetX(), row_[2].GetX(),
                        row_[0].GetY(), row_[1].GetY(), row_[2].GetY(),
                        row_[0].GetZ(), row_[1].GetZ(), row_[2].GetZ());
	}
	
private:
    /// @brief Calculate the matrix cofactor 
    ///@param r1 The first row to use for calculating the cofactor
    ///@param c1 The first column to use for calculating the cofactor
    /// @param r2 The second row to use for calculating the cofactor
    /// @param c2 The second column to use for calculating the cofactor
    /// See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details
    inline double cofac(int r1, int c1, int r2, int c2) const 
    {
        return row_[r1][c1] * row_[r2][c2] - row_[r1][c2] * row_[r2][c1];
    }
    /// データ
    Vector3 row_[3];
};

inline Vector3 operator*(const Matrix33& m, const Vector3& v) 
{
    return Vector3(m[0].Dot(v), m[1].Dot(v), m[2].Dot(v));
}

inline Vector3 operator*(const Vector3& v, const Matrix33& m)
{
    return Vector3(m.tdotx(v), m.tdoty(v), m.tdotz(v));
}

inline bool operator==(const Matrix33& m1, const Matrix33& m2)
{
    return ( m1[0][0] == m2[0][0] && m1[1][0] == m2[1][0] && m1[2][0] == m2[2][0] &&
             m1[0][1] == m2[0][1] && m1[1][1] == m2[1][1] && m1[2][1] == m2[2][1] &&
             m1[0][2] == m2[0][2] && m1[1][2] == m2[1][2] && m1[2][2] == m2[2][2] );
}
    
inline Matrix33 operator*(const Matrix33& m1, const Matrix33& m2)
{
    return Matrix33(
        m2.tdotx( m1[0]), m2.tdoty( m1[0]), m2.tdotz( m1[0]),
        m2.tdotx( m1[1]), m2.tdoty( m1[1]), m2.tdotz( m1[1]),
        m2.tdotx( m1[2]), m2.tdoty( m1[2]), m2.tdotz( m1[2]));
}

/// #2f((x y z)(x y z)(x y z))フォーマットで出力する
/// @param [in] stream 出力先ストリーム
/// @param [in] m 出力するベクトル
inline std::ostream& operator<<(std::ostream& stream, const Matrix33& m)
{
    stream << "#2f(";
    stream << "(" << m[0][0] << " " << m[0][1] << " " << m[0][2] << ") ";
    stream << "(" << m[1][0] << " " << m[1][1] << " " << m[1][2] << ") ";
    stream << "(" << m[2][0] << " " << m[2][1] << " " << m[2][2] << ")";
    stream << ")";
    return stream;
}

}

#endif

