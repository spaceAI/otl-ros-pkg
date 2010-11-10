// copyright Takashi Ogura <t.ogura@gmail.com>
// 
//
#ifndef EGEOMETRY_COORDINATES_H
#define EGEOMETRY_COORDINATES_H

#include <egeometry/vector3.h>
#include <egeometry/matrix33.h>
//#include <vector>
//#include <string>

namespace egeometry
{

enum WorldOrLocal
{
    kLocal = 0,
    kWorld,
    kParent,
};

/// convert degree value into radian value
/// @param [in] deg degree value
inline double DegToRad(double deg)
{
    return deg / 180.0 * M_PI;
}

/// convert radian value into degree value
/// @param [in] rad radian value
inline double RadToDeg(double rad)
{
    return rad / M_PI * 180;
}

const FloatVector kAxisX = FloatVector(1, 0, 0);
const FloatVector kAxisY = FloatVector(0, 1, 0);
const FloatVector kAxisZ = FloatVector(0, 0, 1);

/// 座標系を表すクラス。位置と姿勢を持つ。
class Coordinates
{
public:
    /// 固定世界座標
    const static Coordinates world_coords;
    /// デフォルトコンストラクタ
    Coordinates();
    /// 名前で初期化するコンストラクタ
    explicit Coordinates(const std::string &name);
    /// 名前で初期化するコンストラクタ
    explicit Coordinates(const char * const name);
    /// 位置で初期化するコンストラクタ
    explicit Coordinates(const FloatVector &pos);
    /// 位置で初期化するコンストラクタ
    explicit Coordinates(const double * const pos);
    /// 姿勢で初期化するコンストラクタ
    explicit Coordinates(const FloatMatrix &rot);
    /// コピーコンストラクタ
    Coordinates(const Coordinates &coords);
    /// 姿勢、位置、名前で初期化するコンストラクタ
    Coordinates(const FloatMatrix &rot, const FloatVector &pos, const std::string &name="");
    /// 代入
    Coordinates &operator= (const Coordinates &coords);
    /// 現在はなにもしない
    virtual ~Coordinates();
    /// 姿勢を返す
    const FloatMatrix & GetRotation() const;
    /// 位置を返す
    const FloatVector & GetPosition() const;
    /// 上位互換のため。
    inline virtual void Changed(){};
    /// 回転行列の0列を返す
    FloatVector GetXAxis() const;
    /// 回転行列の1列を返す
    FloatVector GetYAxis() const;
    /// 回転行列の2列を返す
    FloatVector GetZAxis() const;
    /// 名前の参照を返す
    const std::string& GetName() const;
    /// 名前を設定する
    void SetName(const std::string &name);
    /// 姿勢を代入する
    void SetRotation(const FloatMatrix &rot);
    /// 姿勢を代入する
    void SetRotation(const double xx, const double xy, const double xz, 
                     const double yx, const double yy, const double yz, 
                     const double zx, const double zy, const double zz);
    /// 位置を代入する
    void SetPosition(const FloatVector &pos);
    /// 位置を代入する
    void SetPosition(const double x, const double y, const double z);

    /// 更新する。euslispのnewcoordsに当たる
    virtual void SetCoords(const FloatMatrix &rot, const FloatVector &pos);
    /// 更新する。euslispのnewcoordsに当たる
    virtual void SetCoords(const Coordinates &coords);
    /// 世界座標系での姿勢を返す
    virtual FloatMatrix GetWorldRotation() const;
    /// 世界座標系での姿勢を返す
    virtual void GetWorldRotation(FloatMatrix &result_mat) const;
    /// 世界座標系での位置を返す
    virtual FloatVector GetWorldPosition() const;
    /// 世界座標系での位置を返す
    virtual void GetWorldPosition(FloatVector &result_vec) const;
    /// 世界座標系での座標系を返す
    virtual const Coordinates &GetWorldCoords() const;
    /// 世界座標系での座標系を返す(copy-worldcoords)
    virtual void GetWorldCoords(Coordinates &result_coords) const;
    /// 初期状態(位置 (0 0 0), 姿勢：単位行列)にする。
    virtual void ResetCoords();
    /// 上位互換。親として世界座標系を返す
    virtual const Coordinates &GetParent() const;
    /// この座標系での位置を世界座標系に直す
    virtual void GetTransformVector(const FloatVector &pos, FloatVector &result_pos) const;
    /// 世界座標系での位置をこの座標系での位置に直す
    virtual void GetInverseTransformVector(const FloatVector &pos, FloatVector &result_vector) const;
    /// 指定した軸回りにこの座標系を回転させる。
    virtual void Rotate(const double theta, const FloatVector &axis, const WorldOrLocal local=kLocal);
    /// 指定した軸回りに回転行列に姿勢をセットする
    virtual void Orient(const double theta, const FloatVector &axis, const WorldOrLocal local=kLocal);
    /// 相対的に位置を移動させる。local==trueならこの座標基準。falseなら世界座標基準
    virtual void Translate (const FloatVector &vector, const WorldOrLocal local=kLocal);
    /// 絶対的に位置を移動させる。local==trueならこの座標基準。falseなら世界座標基準
    virtual void Locate (const FloatVector &vector, const WorldOrLocal local=kLocal);
    
    virtual void GetParentVector(const FloatVector &vector, FloatVector &result_vector, const WorldOrLocal local=kLocal) const;
    virtual void GetParentOrientation(const FloatVector &vector, FloatVector &result_vector, const WorldOrLocal local=kLocal) const;

    /// RPYで姿勢を指定する
    void SetRPY(const double r, const double p, const double y);
    /// RPY角を取得する
    void GetRPY(double &r, double &p, double &y, const bool result1=true) const;
    /// quaternionで姿勢を指定する
    virtual void SetQuaternion(const double* const q);
    /// 姿勢をquaternionで取得する
    void GetQuaternion(double *q) const;
    /// 座標系を移動する
    virtual void MoveTo(const Coordinates &dest_coords, const WorldOrLocal local=kLocal);
    /// 座標系をかける
    virtual void Transform(const Coordinates &coords, Coordinates &result_coords, const WorldOrLocal local=kLocal);
    /// 変形させる座標系を得る
    virtual void GetTransformation(const Coordinates &coords, Coordinates &result_coords, WorldOrLocal local=kLocal) const;
    /// この座標系から見た座標系を返す
    virtual void GetInverseTransformation(Coordinates &coords) const;
    
    //virtual void Transform(const Coordinates &coords, const Coordinates &base_coords, Coordinates &result_coords);

    /*

      virtual void MoveTo(const Coords &dest_coords, const Coords &base_coords);
      virtual void GetRotateVector(const FloatVector &pos, FloatVector &result_pos);

      virtual void GetInverseTransformation(Coords &result_coords) const;
      virtual void GetTransformation(const Coordinates &coords, Coordinates &result_coords, bool local=true) const;
      virtual void Transform(const Coordinates &coords, Coordinates &result_coords, bool local=true);
      virtual void Transform(const Coordinates &coords, const Coordinates &base_coords, Coordinates &result_coords);
      ords);
      //scale

      virtual void GetRotationAngle(FloatVector &result_vector);
      virtual void Get4x4(FloatMatrix &result_matrix);

      protected:
      virtual void RotateWithMatrix(FloatMatrix &matrix, bool local);
      virtual void RotateWithMatrix(FloatMatrix &matrix, const Coordinates &base_cords);
      virtual void OrientWithMatrix(FloatMatrix &matrix, bool local);
      virtual void OrientWithMatrix(FloatMatrix &matrix, const Coordinates &base_coords);
    */
private:
    /// 位置
    FloatMatrix rot_;
    /// 姿勢
    FloatVector pos_;
    /// 名前
    std::string name_;
};

/// 比較演算。位置と姿勢が同じときtrueを返す
bool operator== (const Coordinates &c1, const Coordinates &c2);
/// 比較演算。==の反転
bool operator!= (const Coordinates &c1, const Coordinates &c2);
void TransformCoords(const Coordinates &c1, const Coordinates &c2, Coordinates &c3);
void TransformVector(const Coordinates &c1, const FloatVector &v1, FloatVector &v2);

std::ostream& operator<<(std::ostream& stream, const Coordinates& coord);
    
}


#endif // EGEOMETRY_COORDINATES_H
