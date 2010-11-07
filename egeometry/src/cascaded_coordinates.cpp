#include <egeometry/cascaded_coordinates.h>
#include <algorithm>
#include <stdexcept>

namespace egeometry
{

CascadedCoordinates::CascadedCoordinates():
    parent_(NULL),
    descendants_(),
    worldcoords_(),
    changed_(true)
{
}

CascadedCoordinates::CascadedCoordinates(const std::string &name):
    Coordinates(name),
    parent_(NULL),
    descendants_(),
    worldcoords_(),
    changed_(true)
{
}

CascadedCoordinates::CascadedCoordinates(const char * const name):
    Coordinates(name),
    parent_(NULL),
    descendants_(),
    worldcoords_(),
    changed_(true)
{
}

CascadedCoordinates::CascadedCoordinates(const FloatVector &pos):
    Coordinates(pos),
    parent_(NULL),
    descendants_(),
    worldcoords_(),
    changed_(true)
{
}


CascadedCoordinates::CascadedCoordinates(const double * const pos):
    Coordinates(pos),
    parent_(NULL),
    descendants_(),
    worldcoords_(),
    changed_(true)
{
}

CascadedCoordinates::CascadedCoordinates(const FloatMatrix &rot):
    Coordinates(rot),
    parent_(NULL),
    descendants_(),
    worldcoords_(),
    changed_(true)
{
}

CascadedCoordinates::CascadedCoordinates(const Coordinates &coords):
    Coordinates(coords),
    parent_(NULL),
    descendants_(),
    worldcoords_(),
    changed_(true)
{
}

CascadedCoordinates::CascadedCoordinates(const FloatMatrix &rot, const FloatVector &pos, const std::string &name):
    Coordinates(rot, pos, name),
    parent_(NULL),
    descendants_(),
    worldcoords_(),
    changed_(true)
{
}

CascadedCoordinates::~CascadedCoordinates()
{
}

bool CascadedCoordinates::HasParent() const
{
    return (parent_ != NULL);
}

const CascadedCoordinates &CascadedCoordinates::GetParent() const
{
    if ( !HasParent())
    {
        throw std::runtime_error("has no parent");
    }
    return *parent_;
}

bool CascadedCoordinates::operator== (const CascadedCoordinates &coords)
{
    return (this == &coords);
}

bool CascadedCoordinates::operator!= (const CascadedCoordinates &coords)
{
    return (this != &coords);
}

void CascadedCoordinates::GetParent(CascadedCoordinates &coords) const
{
    if (!HasParent())
    {
        // copy
        coords = *parent_;
    }
}


const CasCoordsPtrVector &CascadedCoordinates::GetDescendants() const
{
    return descendants_;
}

void CascadedCoordinates::Assoc(CascadedCoordinates &child)
{
    std::vector<CascadedCoordinates *>::iterator it;
    it = find(descendants_.begin(), descendants_.end(), &child);
    if (it == descendants_.end()) // not found
    {
        Coordinates c;
        GetWorldCoords().GetTransformation(child.GetWorldCoords(), c);
        child.Obey(*this);
        child.SetCoords(c);
        descendants_.push_back(&child);
    }
}


void CascadedCoordinates::Dissoc(CascadedCoordinates &child)
{
    CasCoordsPtrVector::iterator it;
    it = find(descendants_.begin(), descendants_.end(), &child);
    if (it != descendants_.end()) // found
    {
        CascadedCoordinates c;
        child.GetWorldCoords(c);
        descendants_.erase(std::remove(descendants_.begin(), descendants_.end(), &child), descendants_.end());
        child.Disobey(*this);
        child.SetCoords(c);
    }
}


void CascadedCoordinates::ClearAssoc()
{
    CasCoordsPtrVector::iterator it;
    for (it  = descendants_.begin();
         it != descendants_.end();
         ++it)
    {
        CascadedCoordinates c;
        (*it)->GetWorldCoords(c);
        (*it)->Disobey(*this);
        (*it)->SetCoords(c);
    }
    descendants_.clear();
}

void CascadedCoordinates::Obey(CascadedCoordinates &mother)
{
    if (HasParent())
    {
        parent_->Dissoc(*this);
    }
    parent_ = &mother;
}

void CascadedCoordinates::Disobey(CascadedCoordinates &mother)
{
    if (&mother == parent_)
    {
        parent_ = NULL;
    }
}

void CascadedCoordinates::SetCoords(const FloatMatrix &rot, const FloatVector &pos)
{
    if (&rot == &GetRotation() && &pos == &GetPosition())
    {
        // do nothing
    }
    else
    {
        Coordinates::SetCoords(rot, pos);
    }
    Changed();
}

void CascadedCoordinates::SetCoords(const Coordinates &coords)
{
    if (&coords != this)
    {
        Coordinates::SetCoords(coords);
    }
    Changed();
}

void CascadedCoordinates::Changed()
{
    if (!changed_)
    {
        changed_ = true;
        CasCoordsPtrVector::iterator it;
        for (it  = descendants_.begin();
             it != descendants_.end();
             ++it)
        {
            (*it)->Changed();
        }
    }
}

FloatMatrix CascadedCoordinates::GetWorldRotation() const
{
    return GetWorldCoords().GetRotation();
}

// void CascadedCoordinates::GetWorldRotation(FloatMatrix &rot) const
// {
//     GetWorldCoords().GetRotation(rot);
// }

FloatVector CascadedCoordinates::GetWorldPosition() const
{
    return GetWorldCoords().GetPosition();
}

// void CascadedCoordinates::GetWorldPosition(FloatVector &pos) const
// {
//     GetWorldCoords().GetPosition(pos);
// }


void CascadedCoordinates::GetWorldCoords(Coordinates &coords) const
{
    if (changed_)
    {
        if (HasParent())
        {
            TransformCoords(parent_->GetWorldCoords(), *this, worldcoords_);
        }
        else
        {
            worldcoords_.SetCoords(*this);
        }
        //Update();
        changed_ = false;
    }
    coords = worldcoords_;
}

const Coordinates &CascadedCoordinates::GetWorldCoords() const
{
    if (changed_)
    {
        if (HasParent())
        {
            TransformCoords(parent_->GetWorldCoords(), *this, worldcoords_);
        }
        else
        {
            worldcoords_.SetCoords(*this);
        }
        //Update();
        changed_ = false;
    }
    return worldcoords_;
}

void CascadedCoordinates::Update()
{
}

void CascadedCoordinates::GetTransformVector(const FloatVector &pos, FloatVector &result_vector) const
{
    GetWorldCoords().GetTransformVector(pos, result_vector);
}

void CascadedCoordinates::GetInverseTransformVector(const FloatVector &pos, FloatVector &result_vector) const
{
    GetWorldCoords().GetInverseTransformVector(pos, result_vector);
}

void CascadedCoordinates::GetInverseTransformation(Coordinates &coords) const
{
    GetWorldCoords().GetInverseTransformation(coords);
}

void CascadedCoordinates::GetTransformation(const Coordinates &coords, Coordinates &result_coords, bool local) const
{
    Coordinates w1inv, w2;
    coords.GetWorldCoords(w2);
    w2.GetInverseTransformation(w1inv);
    if (local)
    {
        TransformCoords(w1inv, w2, result_coords);
    }
    else
    {
        // world
        TransformCoords(w2, w1inv, result_coords);
    }
}


void CascadedCoordinates::Transform(const Coordinates &coords, Coordinates &result_coords, bool local)
{
    if (local)
    {
        TransformCoords(*this, coords, *this);
    }
    else
    {
        //world
        TransformCoords(GetParent(), *this, *this);
        TransformCoords(coords, *this, *this);
        Coordinates c;
        GetParent().GetInverseTransformation(c);
        TransformCoords(c, *this, *this);
    }
}


void CascadedCoordinates::MoveTo(const Coordinates &dest_coords, bool local)
{
    if (local)
    {
        TransformCoords(*this, dest_coords, *this);
        SetCoords(*this);
    }
    else
    {//world
        Coordinates c;
        GetParent().GetInverseTransformation(c);
        TransformCoords(c, dest_coords, c);
        SetCoords(c);
    }
}


}
