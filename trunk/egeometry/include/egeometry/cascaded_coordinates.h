#ifndef EUSGEOMETRY_CASCADED_COORDINATES_H
#define EUSGEOMETRY_CASCADED_COORDINATES_H

#include <egeometry/coordinates.h>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace egeometry
{

class CascadedCoordinates;


class CascadedCoordinates : public Coordinates
{
public:
    CascadedCoordinates();
    explicit CascadedCoordinates(const std::string &name);
    explicit CascadedCoordinates(const FloatVector &pos);
    explicit CascadedCoordinates(const float * const pos);
    explicit CascadedCoordinates(const double * const pos);
    explicit CascadedCoordinates(const FloatMatrix &rot);
    explicit CascadedCoordinates(const Coordinates &coords);
    CascadedCoordinates(const FloatMatrix &rot, const FloatVector &pos, const std::string &name="");

    bool operator== (const CascadedCoordinates &coords);
    bool operator!= (const CascadedCoordinates &coords);

    ~CascadedCoordinates();
    const std::vector<CascadedCoordinates *> &GetDescendants() const;
    void Assoc(CascadedCoordinates &child);
    void Dissoc(CascadedCoordinates &child);
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
    CascadedCoordinates* parent_; // shared_ptrのやり方が分からないので暫定
    std::vector<CascadedCoordinates *> descendants_; // shared_ptrのやり方が分からないので暫定
    mutable Coordinates worldcoords_;
    // manager
    mutable bool changed_;
};

typedef boost::shared_ptr<CascadedCoordinates> CasCoordsPtr;

}

#endif //CASCADED_COORDINATES_H
