////////////////////////////////////////////////
// egeometry - coordinates.cpp
//
// BSD License
//
// ogutti <t.ogura@gmail.com>
//
// version 0.0.1 2010/10/26
//
////////////////////////////////////////////////

#include <egeometry/coordinates.h>

namespace egeometry
{
  
const Coordinates Coordinates::world_coords("world_coords");

Coordinates::Coordinates():
    rot_(Matrix33::GetIdentity()), pos_(), name_("")
{
}
  
Coordinates::Coordinates(const double* const pos):
    rot_(Matrix33::GetIdentity()), pos_(pos), name_("")
{
}

Coordinates::Coordinates(const FloatVector &pos):
    rot_(Matrix33::GetIdentity()), pos_(pos), name_("")
{
}


Coordinates::Coordinates(const Coordinates &coords):
    rot_(coords.GetRotation()), pos_(coords.GetPosition()), name_("")
{
}

Coordinates::Coordinates(const FloatMatrix &rot, const FloatVector &pos, const std::string &name):
    rot_(rot), pos_(pos), name_(name)
{
}

Coordinates::Coordinates(const FloatMatrix &rot):
    rot_(rot), pos_(), name_("")
{
}

Coordinates::Coordinates(const std::string &name):
    rot_(Matrix33::GetIdentity()), pos_(), name_(name)
{
}

Coordinates::Coordinates(const char * const name):
    rot_(Matrix33::GetIdentity()), pos_(), name_(name)
{
}

Coordinates & Coordinates::operator= (const Coordinates &coords)
{
    name_ = "copy_" + coords.GetName();
    pos_ = coords.GetPosition();
    rot_ = coords.GetRotation();
    return *this;
}

bool operator== (const Coordinates &c1, const Coordinates &c2)
{
    return (c1.GetRotation() == c2.GetRotation() &&
            c1.GetPosition() == c2.GetPosition());
}

bool operator!= (const Coordinates &c1, const Coordinates &c2)
{
    return !(c1==c2);
}

const FloatMatrix & Coordinates::GetRotation() const
{
    return rot_;
}
  
const FloatVector & Coordinates::GetPosition() const
{
    return pos_;
}

#if 0
void Coordinates::GetRotation(FloatMatrix &result_matrix) const
{
    // コピー
    result_matrix = rot_;
}
  
void Coordinates::GetPosition(FloatVector &result_vector) const
{
    result_vector = pos_;
}

#endif

FloatVector Coordinates::GetXAxis() const
{
    return rot_.GetColumn(0);
}

FloatVector Coordinates::GetYAxis() const
{
    return rot_.GetColumn(1);
}

FloatVector Coordinates::GetZAxis() const
{
    return rot_.GetColumn(2);
}

const std::string &Coordinates::GetName() const
{
    return name_;
}

void Coordinates::SetName(const std::string &name)
{
    name_ = name;
}

void Coordinates::SetRotation(const FloatMatrix &rot)
{
    rot_ = rot;
    Changed();
}

void Coordinates::SetRotation(const double xx, const double xy, const double xz, 
                              const double yx, const double yy, const double yz, 
                              const double zx, const double zy, const double zz)
{
    rot_.SetValue(xx, xy, xz,
                  yx, yy, yz,
                  zx, zy, zz);
    Changed();
}

void Coordinates::SetPosition(const FloatVector &pos)
{
    pos_ = pos;
    Changed();
}

void Coordinates::SetPosition(const double x, const double y, const double z)
{
    pos_.SetValue(x, y, z);
    Changed();
}


void Coordinates::MoveTo(const Coordinates &dest_coords, const WorldOrLocal local)
{
    if (local == kLocal)
    {
        TransformCoords(*this, dest_coords, *this);
        Changed();
    }
    else
    {
        SetCoords(dest_coords);
    }
}


void Coordinates::Transform(const Coordinates &coords, Coordinates &result_coords, const WorldOrLocal local)
{
    if (local == kLocal)
    {
        TransformCoords(*this, coords, result_coords);
    }
    else
    {// world
        TransformCoords(coords, *this, result_coords);
    }
    Changed();
}

void Coordinates::GetTransformation(const Coordinates &coords, Coordinates &result_coords, const WorldOrLocal local) const
{
    Coordinates c1, c2;
    coords.GetWorldCoords(c2);
    GetWorldCoords(c1);
    c1.GetInverseTransformation(result_coords);
    if (local == kLocal)
    {
        TransformCoords(result_coords, c2, result_coords);
    }
    else
    {
        TransformCoords(c2, result_coords, result_coords);
    }
}

void Coordinates::GetInverseTransformation(Coordinates &coords) const
{
    coords.SetRotation(rot_.Transpose());
    coords.SetPosition(coords.GetRotation() * pos_);
    coords.SetPosition(-1 * coords.GetPosition());
}

/*  
//virtual void ReplaceCoords(const Coordinates &coords);
virtual void SetCoords(const FloatVector &rot, const FloatVector &pos);
  
virtual const FloatVector GetWorldRot();
virtual const FloatVector GetWorldPos();
virtual bool IsChanged();
virtual void ResetCoords();
virtual void MoveTo(const Coords &dest_coords, bool local=true);
virtual void MoveTo(const Coords &dest_coords, const Coords &base_coords);
virtual void GetRotateVector(const FloatVector &pos, FloatVector &result_pos);
virtual void TransformVector(const FloatVector pos, FloatVector result_pos);
virtual void InverseTransformVector(const FloatVector &pos, FloatVector &result_vector);
virtual void GetInverseTransformation(Coords &result_coords) const;
virtual void GetTransformation(const Coordinates &coords, Coordinates &result_coords, bool local=true) const;
virtual void Transform(const Coordinates &coords, Coordinates &result_coords, bool local=true);
virtual void Transform(const Coordinates &coords, const Coordinates &base_coords, Coordinates &result_coords);
ords);
virtual Void Rotate(const double theta, const FloatVector &axis, bool local=true);
virtual Void Orient(const double theta, const FloatVector &axis, bool local=true);
virtual Void GetParentVector(const FloatVector &vector, FloatVector &result_vector, bool local=true) const;
virtual Void GetParentOrientation(const FloatVector &vector, FloatVector &result_vector, bool local=true) const;
virtual void Translate (const FloatVector &vector, bool local=true);
virtual void Locate (const FloatVector &vector, bool local=true);
//scale
virtual void SetEular(const double azimuth, const double elevation, const double rotation);
virtual void GetEularAngle(FloatVector &result_vector);
virtual void SetRPY(const double r, const double p, const double y);
virtual void GetRotationAngle(FloatVector &result_vector);
virtual void Get4x4(FloatVector &result_vector);
 
protected:

virtual void RotateWithMatrix(FloatVector &matrix, bool local);
virtual void RotateWithMatrix(FloatVector &matrix, const Coordinates &base_cords);
virtual void OrientWithMatrix(FloatVector &matrix, bool local);
virtual void OrientWithMatrix(FloatVector &matrix, const Coordinates &base_coords);
*/

std::ostream& operator<<(std::ostream& stream, const Coordinates& coord)
{
    stream << coord.GetName() << coord.GetPosition() << " " << coord.GetRotation();
    return stream;
}

#if 0
std::istream& operator>>(std::istream& stream, Coordinates& coord)
{
    stream >> coord.name_ >> coord.pos_ >> coord.rot_;
    return stream;
}
#endif

// newcoords
void Coordinates::SetCoords(const FloatMatrix &rot, const FloatVector &pos)
{
    SetRotation(rot);
    SetPosition(pos);
}

void Coordinates::SetCoords(const Coordinates &coords)
{
    SetPosition(coords.GetPosition());
    SetRotation(coords.GetRotation());
}


FloatMatrix Coordinates::GetWorldRotation() const
{
    return rot_;
}

void Coordinates::GetWorldRotation(FloatMatrix &result_mat) const
{
    result_mat = rot_;
}
  
FloatVector Coordinates::GetWorldPosition() const
{
    return pos_;
}

void Coordinates::GetWorldPosition(FloatVector &result_vec) const
{
    result_vec = pos_;
}

const Coordinates &Coordinates::GetWorldCoords() const
{
    return *this;
}


void Coordinates::GetWorldCoords(Coordinates &result_coords) const
{
    result_coords = *this;
}


void Coordinates::ResetCoords()
{
    pos_.SetZero();
    rot_ = FloatMatrix::GetIdentity();
    Changed();
}

#if 0
void GetInverseMatrix(const FloatMatrix &mat, FloatMatrix &ret)
{
    ret = identity_matrix<float>(ret.size1());
    FloatMatrix copy_lh(mat);
    permutation_matrix<> pm(mat.size1());
    //lu_factorize(mat, pm);
    //lu_substitute(mat, pm, ret);
}
#endif

const Coordinates &Coordinates::GetParent() const
{
    return world_coords;
}
  
void Coordinates::GetTransformVector(const FloatVector &pos, FloatVector &result_pos) const
{
    result_pos = (rot_ * pos) + pos_;
}
  
void Coordinates::GetInverseTransformVector(const FloatVector &pos, FloatVector &result_pos) const
{
    FloatMatrix inv_rot = rot_.Inverse();
    result_pos = inv_rot * pos - inv_rot * pos_;
}

void Coordinates::RotateWithAxis(const double theta, const FloatVector &axis, const WorldOrLocal local)
{
    FloatMatrix rotation_mat;
    SetRotationMatrixWithAxis(theta, axis, rotation_mat);
    if (local == kLocal)
    {
        rot_ = rot_ * rotation_mat;
    }
    else
    {
        // parent
        rot_ = rotation_mat * rot_;
        // todo world
    }
    SetCoords(rot_, pos_);
}

void Coordinates::OrientWithAxis(const double theta, const FloatVector &axis, const WorldOrLocal local)
{
    FloatMatrix rotation_mat;
    SetRotationMatrixWithAxis(theta, axis, rotation_mat);
    if (local == kLocal)
    {
        rot_ = rot_ * rotation_mat;
    }
    else
    {
        rot_ = rotation_mat;
    }
    SetCoords(rot_, pos_);
}

// only for internal use  
void SetRotationMatrixWithAxis(const double theta, const FloatVector axis, FloatMatrix &mat)
{
    Vector3 normalized_axis(axis);
    normalized_axis.Normalize();
    
    double ct1 = 1-cos(theta);
    double ct = cos(theta);
    double st = sin(theta);

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


void Coordinates::GetParentOrientation(const FloatVector &vector, FloatVector &result_vector, const WorldOrLocal local) const
{
    if (local == kLocal)
    {
        result_vector = rot_ * vector;
    }
    else
    {
        result_vector = vector;
    }
}

void Coordinates::GetParentVector(const FloatVector &vector, FloatVector &result_vector, const WorldOrLocal local) const
{
    if (local == kLocal)
    {
        GetTransformVector(vector, result_vector);
    }
    else
    {
        result_vector = vector;
    }
}

void Coordinates::Translate (const FloatVector &vector, const WorldOrLocal local)
{
    FloatVector result;
    GetParentOrientation(vector, result, local);
    pos_ += result;
    SetCoords(rot_, pos_);
}

void Coordinates::Locate (const FloatVector &vector, const WorldOrLocal local)
{
    GetParentVector(vector, pos_, local);
    Changed();
}

void Coordinates::GetRPY(double &r, double &p, double &y, const bool result1) const
{
    double a, b, c, sa, ca;

    a = atan2(rot_[1][0],rot_[0][0]);
    sa = sin(a); ca = cos(a);
    b = atan2(-rot_[2][0], ca*rot_[0][0] + sa*rot_[1][0]);
    c = atan2(sa*rot_[0][2] - ca*rot_[1][2], -sa*rot_[0][1] + ca*rot_[1][1]);
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
        b = atan2(-rot_[2][0], ca*rot_[0][0] + sa*rot_[1][0]);
        c = atan2(sa*rot_[0][2] - ca*rot_[1][2], -sa*rot_[0][1] + ca*rot_[1][1]);
        
        r = a;
        p = b;
        y = c;
    }
}


void Coordinates::SetRPY(const double r, const double p, const double y)
{
    double cr = cos(r);
    double sr = sin(r);
    double cp = cos(p);
    double sp = sin(p);
    double cy = cos(y);
    double sy = sin(y);
    rot_[0][0] = cr * cp;
    rot_[0][1] = cr * sp * sy - sr * cy;
    rot_[0][2] = cr * sp * cy + sr * sy;
    rot_[1][0] = sr * cp;
    rot_[1][1] = sr * sp * sy + cr * cy;
    rot_[1][2] = sr * sp * cy - cr * sy;
    rot_[2][0] = -sp;
    rot_[2][1] = cp * sy;
    rot_[2][2] = cp * cy;
    Changed();
}

// q[4] = {x, y, z, w}
void Coordinates::SetQuaternion(const double* const q)
{
    if (q == NULL)
    {
        return;
    }
    rot_[0][0] = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    rot_[0][1] = 2.0 * (q[0] * q[1] + q[3] * q[2]);
    rot_[0][2] = 2.0 * (q[0] * q[2] - q[3] * q[1]);
    rot_[1][0] = 2.0 * (q[0] * q[1] - q[3] * q[2]);
    rot_[1][1] = 1.0 - 2.0 * (q[0] * q[0] + q[2] * q[2]);
    rot_[1][2] = 2.0 * (q[1] * q[2] + q[3] * q[0]);
    rot_[2][0] = 2.0 * (q[0] * q[2] + q[3] * q[1]);
    rot_[2][1] = 2.0 * (q[1] * q[2] - q[3] * q[0]);
    rot_[2][2] = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);

    Changed();
}

Coordinates::~Coordinates()
{
    
}

void Coordinates::GetQuaternion(double *q) const
{
    if ( q == NULL)
    {
        return;
    }
    
    double s;
    double trace = rot_[0][0] + rot_[1][1] + rot_[2][2] + 1.0;
    if (trace >= 1.0f) {
        s = 0.5f / sqrtf(trace);
        q[0] = (rot_[1][2] - rot_[2][1]) * s;
        q[1] = (rot_[2][0] - rot_[0][2]) * s;
        q[2] = (rot_[0][1] - rot_[1][0]) * s;
        q[3] = 0.25f / s; // w
    }
    else {
        double max = rot_[1][1] > rot_[2][2] ? rot_[1][1] : rot_[2][2];
        if (max < rot_[0][0]) { // ?
            s = sqrtf(rot_[0][0] - (rot_[1][1] + rot_[2][2]) + 1.0f);
            q[0] = s * 0.5f;
            s = 0.5f / s;
            q[1]= (rot_[0][1] + rot_[1][0]) * s;
            q[2] = (rot_[2][0] + rot_[0][2]) * s;
            q[3] = (rot_[1][2] - rot_[2][1]) * s; // w
        }
        else if (max == rot_[1][1]) {
            s = sqrtf(rot_[1][1] - (rot_[2][2] + rot_[0][0]) + 1.0f);
            q[1] = s * 0.5f;
            s = 0.5f / s;
            q[0] = (rot_[0][1] + rot_[1][0]) * s;
            q[2] = (rot_[1][2] + rot_[2][1]) * s;
            q[3] = (rot_[2][0] - rot_[0][2]) * s;
        }
        else {
            s = sqrtf(rot_[2][2] - (rot_[0][0] + rot_[1][1]) + 1.0f);
            q[2] = s * 0.5f;
            s = 0.5 / s;
            q[0] = (rot_[2][0] + rot_[0][2]) * s;
            q[1] = (rot_[1][2] + rot_[2][1]) * s;
            q[3] = (rot_[0][1] - rot_[1][0]) * s;
        }
    }
}

void TransformCoords(const Coordinates &c1, const Coordinates &c2, Coordinates &c3)
{
    c3.SetPosition(c1.GetPosition() + c1.GetRotation() * c2.GetPosition());
    c3.SetRotation(c1.GetRotation() * c2.GetRotation());
}
    
void TransformVector(const Coordinates &c1, const FloatVector &v1, FloatVector &v2)
{
    v2 = c1.GetRotation() * v1 + c1.GetPosition();
}

}
