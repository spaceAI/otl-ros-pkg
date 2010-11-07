#ifndef EGEOMETRY_MATRIX33_H
#define EGEOMETRY_MATRIX33_H

#include <egeometry/vector3.h>
#include <iostream>



namespace egeometry
{

/// @brief 3x3の行列。回転行列に用いることを想定している。
/// コードはbtMatrix3x3を非常に参考にしています。というよりコピーに近い。
template <class T>
class Matrix33
{
public:
    /// デフォルトコンストラクタ。すべての要素は0
    inline Matrix33<T>()
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
    inline Matrix33<T>(const T xx, const T xy, const T xz,
                       const T yx, const T yy, const T yz,
                       const T zx, const T zy, const T zz)
    {
        SetValue(xx, xy, xz,
                 yx, yy, yz,
                 zx, zy, zz);
    };

    /// コピーコンストラクタ。値がコピーされる。
    /// @param [in] other コピー元Matrix
    inline Matrix33<T> (const Matrix33<T>& other)
    {
        row_[0] = other.row_[0];
        row_[1] = other.row_[1];
        row_[2] = other.row_[2];
    }

    /// 代入演算子。値がコピーされる。
    /// @param [in] other コピー元Matrix
    inline Matrix33<T>& operator=(const Matrix33<T>& other)
    {
        row_[0] = other.row_[0];
        row_[1] = other.row_[1];
        row_[2] = other.row_[2];
        return *this;
    }

    /// 縦ベクトルを取得する
    /// @param [in] i 列番号
    inline Vector3<T> GetColumn(int i) const
    {
        assert(0 <= i && i < 3);
        return Vector3<T>(row_[0][i],row_[1][i],row_[2][i]);
    }

    /// 横ベクトルを取得する
    /// @param [in] i 行番号
    inline Vector3<T> GetRow(int i) const
    {
        assert(0 <= i && i < 3);
        return row_[i];
    }

    /// 横ベクトルを取得する
    /// @param [in] i 行番号
    inline Vector3<T>& operator[](int i)
    { 
        assert(0 <= i && i < 3);
        return row_[i]; 
    }

    /// 横ベクトルを取得する。const用
    /// @param [in] i 行番号
    inline const Vector3<T>& operator[](int i) const
    { 
        assert(0 <= i && i < 3);
        return row_[i]; 
    }

    /// 行列を右からかける
    /// @param [in] m かける行列
    inline Matrix33<T>& operator*=(const Matrix33<T>& m)
    {
		SetValue(m.tdotx(row_[0]), m.tdoty(row_[0]), m.tdotz(row_[0]),
				 m.tdotx(row_[1]), m.tdoty(row_[1]), m.tdotz(row_[1]),
				 m.tdotx(row_[2]), m.tdoty(row_[2]), m.tdotz(row_[2]));
		return *this;
    };

    /// 0列との内積を計算する
    /// @param [in] v かけるベクトル
    inline T tdotx(const Vector3<T>& v) const 
    {
        return row_[0].GetX() * v.GetX() + row_[1].GetX() * v.GetY() + row_[2].GetX() * v.GetZ();
    }

    /// 1列との内積を計算する
    /// @param [in] v かけるベクトル
    inline T tdoty(const Vector3<T>& v) const 
    {
        return row_[0].GetY() * v.GetX() + row_[1].GetY() * v.GetY() + row_[2].GetY() * v.GetZ();
    }

    /// 2列との内積を計算する
    /// @param [in] v かけるベクトル
    inline T tdotz(const Vector3<T>& v) const 
    {
        return row_[0].GetZ() * v.GetX() + row_[1].GetZ() * v.GetY() + row_[2].GetZ() * v.GetZ();
    }

    /// 逆行列を返す
	inline Matrix33<T> Inverse() const
	{
		Vector3<T> co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
		T det = (*this)[0].Dot(co);
		assert(det != 0.0);
		T s = 1.0 / det;
		return Matrix33<T>(co.GetX() * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s,
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
    inline void SetValue(const T xx, const T xy, const T xz, 
                         const T yx, const T yy, const T yz, 
                         const T zx, const T zy, const T zz)
    {
        row_[0].SetValue(xx,xy,xz);
        row_[1].SetValue(yx,yy,yz);
        row_[2].SetValue(zx,zy,zz);
    }

    /// 単位行列を返す
    inline static const Matrix33<T>& GetIdentity()
    {
        static const Matrix33<T> identityMatrix((1.0), (0.0), (0.0), 
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
    inline Matrix33<T> Transpose() const 
    {
      return Matrix33<T>(row_[0].GetX(), row_[1].GetX(), row_[2].GetX(),
		      row_[0].GetY(), row_[1].GetY(), row_[2].GetY(),
		      row_[0].GetZ(), row_[1].GetZ(), row_[2].GetZ());
    }
    
    inline void SetQuaternion(const T *const q)
    {
        if (q == NULL)
        {
            return;
        }
        row_[0][0] = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
        row_[0][1] = 2.0 * (q[0] * q[1] + q[3] * q[2]);
        row_[0][2] = 2.0 * (q[0] * q[2] - q[3] * q[1]);
        row_[1][0] = 2.0 * (q[0] * q[1] - q[3] * q[2]);
        row_[1][1] = 1.0 - 2.0 * (q[0] * q[0] + q[2] * q[2]);
        row_[1][2] = 2.0 * (q[1] * q[2] + q[3] * q[0]);
        row_[2][0] = 2.0 * (q[0] * q[2] + q[3] * q[1]);
        row_[2][1] = 2.0 * (q[1] * q[2] - q[3] * q[0]);
        row_[2][2] = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
    }

    inline void GetQuaternion(T *q) const
    {
        if ( q == NULL)
        {
            return;
        }
    
        T s;
        T trace = row_[0][0] + row_[1][1] + row_[2][2] + 1.0;
        if (trace >= 1.0f) {
            s = 0.5f / sqrtf(trace);
            q[0] = (row_[1][2] - row_[2][1]) * s;
            q[1] = (row_[2][0] - row_[0][2]) * s;
            q[2] = (row_[0][1] - row_[1][0]) * s;
            q[3] = 0.25f / s; // w
        }
        else {
            T max = row_[1][1] > row_[2][2] ? row_[1][1] : row_[2][2];
            if (max < row_[0][0]) { // ?
                s = sqrtf(row_[0][0] - (row_[1][1] + row_[2][2]) + 1.0f);
                q[0] = s * 0.5f;
                s = 0.5f / s;
                q[1]= (row_[0][1] + row_[1][0]) * s;
                q[2] = (row_[2][0] + row_[0][2]) * s;
                q[3] = (row_[1][2] - row_[2][1]) * s; // w
            }
            else if (max == row_[1][1]) {
                s = sqrtf(row_[1][1] - (row_[2][2] + row_[0][0]) + 1.0f);
                q[1] = s * 0.5f;
                s = 0.5f / s;
                q[0] = (row_[0][1] + row_[1][0]) * s;
                q[2] = (row_[1][2] + row_[2][1]) * s;
                q[3] = (row_[2][0] - row_[0][2]) * s;
            }
            else {
                s = sqrtf(row_[2][2] - (row_[0][0] + row_[1][1]) + 1.0f);
                q[2] = s * 0.5f;
                s = 0.5 / s;
                q[0] = (row_[2][0] + row_[0][2]) * s;
                q[1] = (row_[1][2] + row_[2][1]) * s;
                q[3] = (row_[0][1] - row_[1][0]) * s;
            }
        }
    }


private:
    /// @brief Calculate the matrix cofactor 
    ///@param r1 The first row to use for calculating the cofactor
    ///@param c1 The first column to use for calculating the cofactor
    /// @param r2 The second row to use for calculating the cofactor
    /// @param c2 The second column to use for calculating the cofactor
    /// See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details
    inline T cofac(int r1, int c1, int r2, int c2) const 
    {
        return row_[r1][c1] * row_[r2][c2] - row_[r1][c2] * row_[r2][c1];
    }
    /// データ
    Vector3<T> row_[3];
};

template <class T>
inline Vector3<T> operator*(const Matrix33<T>& m, const Vector3<T>& v) 
{
    return Vector3<T>(m[0].Dot(v), m[1].Dot(v), m[2].Dot(v));
}

template <class T>
inline Vector3<T> operator*(const Vector3<T>& v, const Matrix33<T>& m)
{
    return Vector3<T>(m.tdotx(v), m.tdoty(v), m.tdotz(v));
}

template <class T>
inline bool operator==(const Matrix33<T>& m1, const Matrix33<T>& m2)
{
    return ( m1[0][0] == m2[0][0] && m1[1][0] == m2[1][0] && m1[2][0] == m2[2][0] &&
             m1[0][1] == m2[0][1] && m1[1][1] == m2[1][1] && m1[2][1] == m2[2][1] &&
             m1[0][2] == m2[0][2] && m1[1][2] == m2[1][2] && m1[2][2] == m2[2][2] );
}
template <class T>    
inline Matrix33<T> operator*(const Matrix33<T>& m1, const Matrix33<T>& m2)
{
    return Matrix33<T>(
        m2.tdotx( m1[0]), m2.tdoty( m1[0]), m2.tdotz( m1[0]),
        m2.tdotx( m1[1]), m2.tdoty( m1[1]), m2.tdotz( m1[1]),
        m2.tdotx( m1[2]), m2.tdoty( m1[2]), m2.tdotz( m1[2]));
}

/// #2f((x y z)(x y z)(x y z))フォーマットで出力する
/// @param [in] stream 出力先ストリーム
/// @param [in] m 出力するベクトル
template <class T>
inline std::ostream& operator<<(std::ostream& stream, const Matrix33<T>& m)
{
    stream << "#2f(";
    stream << "(" << m[0][0] << " " << m[0][1] << " " << m[0][2] << ") ";
    stream << "(" << m[1][0] << " " << m[1][1] << " " << m[1][2] << ") ";
    stream << "(" << m[2][0] << " " << m[2][1] << " " << m[2][2] << ")";
    stream << ")";
    return stream;
}

/// RPYをセットする
template <class T>
inline void SetRPYAngles(Matrix33<T> &mat, const T r, const T p, const double y)
{
    double cr = cos(r);
    double sr = sin(r);
    double cp = cos(p);
    double sp = sin(p);
    double cy = cos(y);
    double sy = sin(y);
    mat[0][0] = cr * cp;
    mat[0][1] = cr * sp * sy - sr * cy;
    mat[0][2] = cr * sp * cy + sr * sy;
    mat[1][0] = sr * cp;
    mat[1][1] = sr * sp * sy + cr * cy;
    mat[1][2] = sr * sp * cy - cr * sy;
    mat[2][0] = -sp;
    mat[2][1] = cp * sy;
    mat[2][2] = cp * cy;
}

/// RPYをセットする
template <class T>
inline void GetRPYAngles(const Matrix33<T> &mat, T &r, T &p, T &y, const bool result1=true)
{
    T a, b, c, sa, ca;

    a = atan2(mat[1][0],mat[0][0]);
    sa = sin(a); ca = cos(a);
    b = atan2(-mat[2][0], ca*mat[0][0] + sa*mat[1][0]);
    c = atan2(sa*mat[0][2] - ca*mat[1][2], -sa*mat[0][1] + ca*mat[1][1]);
    if (result1)
    {
        r = a;
        p = b;
        y = c;
    }
    else
    {
        a=a + M_PI;
        sa = sin(a); ca = cos(a);
        b = atan2(-mat[2][0], ca*mat[0][0] + sa*mat[1][0]);
        c = atan2(sa*mat[0][2] - ca*mat[1][2], -sa*mat[0][1] + ca*mat[1][1]);
        
        r = a;
        p = b;
        y = c;
    }
}

/// 回転行列をつくる
template <class T>
inline void SetRotationMatrixWithAxis(const T theta, const Vector3<T> axis, Matrix33<T> &mat)
{
    Vector3<T> normalized_axis(axis);
    normalized_axis.Normalize();
    
    T ct1 = 1-cos(theta);
    T ct = cos(theta);
    T st = sin(theta);

    mat[0][0] = normalized_axis[0] * normalized_axis[0] * ct1 + ct;
    mat[0][1] = normalized_axis[0] * normalized_axis[1] * ct1 - normalized_axis[2] * st;
    mat[0][2] = normalized_axis[2] * normalized_axis[0] * ct1 + normalized_axis[1] * st;
    
    mat[1][0] = normalized_axis[0] * normalized_axis[1] * ct1 + normalized_axis[2] * st;
    mat[1][1] = normalized_axis[1] * normalized_axis[1] * ct1 + ct;
    mat[1][2] = normalized_axis[1] * normalized_axis[2] * ct1 - normalized_axis[0] * st;
    
    mat[2][0] = normalized_axis[2] * normalized_axis[0] * ct1 - normalized_axis[1] * st;
    mat[2][1] = normalized_axis[1] * normalized_axis[2] * ct1 + normalized_axis[0] * st;
    mat[2][2] = normalized_axis[2] * normalized_axis[2] * ct1 + ct;
}

typedef Matrix33<double> FloatMatrix;

}

#endif

