#ifndef EGEOMETRY_CASCADED_COORDINATES_H
#define EGEOMETRY_CASCADED_COORDINATES_H

#include <egeometry/coordinates.h>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace egeometry
{

class CascadedCoordinates;

typedef CascadedCoordinates CasCoords;
typedef CascadedCoordinates* CasCoordsPtr;
//typedef boost::shared_ptr<CascadedCoordinates> CasCoordsPtr;
typedef std::vector<CasCoordsPtr> CasCoordsPtrVector;

/// @brief 連結座標系
class CascadedCoordinates : public Coordinates
{
public:
    CascadedCoordinates();
    explicit CascadedCoordinates(const std::string &name);
    /// 名前で初期化するコンストラクタ
    explicit CascadedCoordinates(const char * const name);
    explicit CascadedCoordinates(const FloatVector &pos);
    explicit CascadedCoordinates(const float * const pos);
    explicit CascadedCoordinates(const double * const pos);
    explicit CascadedCoordinates(const FloatMatrix &rot);
    explicit CascadedCoordinates(const Coordinates &coords);
    CascadedCoordinates(const FloatMatrix &rot, const FloatVector &pos, const std::string &name="");

    bool operator== (const CascadedCoordinates &coords);
    bool operator!= (const CascadedCoordinates &coords);

    ~CascadedCoordinates();
    const CasCoordsPtrVector &GetDescendants() const;
    // assoc a coordinates
    void Assoc(CascadedCoordinates &child);
    // release a child
    void Dissoc(CascadedCoordinates &child);
    /// release all descendants
    void ClearAssoc();
    void Obey(CascadedCoordinates &mother);
    void Disobey(CascadedCoordinates &mother);

    void SetCoords(const FloatMatrix &rot, const FloatVector &pos);
    void SetCoords(const Coordinates &coords);

    void Changed();
    FloatMatrix GetWorldRotation() const;
//    void GetWorldRotation(FloatMatrix &rot) const;
    FloatVector GetWorldPosition() const;
//    void GetWorldPosition(FloatVector &pos) const;
    const Coordinates &GetWorldCoords() const;
    void GetWorldCoords(Coordinates &coords) const;
    virtual void Update();
    const CascadedCoordinates &GetParent() const;
    void GetParent(CascadedCoordinates &coords) const;

    void GetTransformVector(const FloatVector &pos, FloatVector &result_pos) const;
    void GetInverseTransformVector(const FloatVector &pos, FloatVector &result_vector) const;
    void GetInverseTransformation(Coordinates &coords) const;
    void GetTransformation(const Coordinates &coords, Coordinates &result_coords, bool local=true) const;
    void Transform(const Coordinates &coords, Coordinates &result_coords, bool local=true);
    void MoveTo(const Coordinates &dest_coords, bool local=true);

    bool HasParent() const;
private:
    CasCoordsPtr parent_;
    std::vector<CasCoordsPtr> descendants_;
    mutable Coordinates worldcoords_;
    mutable bool changed_;
};


}

#endif //EGEOMETRY_CASCADED_COORDINATES_H
