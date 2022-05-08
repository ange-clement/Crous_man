#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

class Rotation {
public:
    glm::mat3 rotationMatrix;

public:    
    Rotation();
    Rotation(glm::mat3 rotationMatrix);

    glm::vec3 rotate(glm::vec3 p);

    void setRotation(float angle, glm::vec3 axis);

    void combineRotation(float angle, glm::vec3 axis);

    Rotation combineRotation(Rotation r);

    Rotation inverse();

    Rotation mixWith(Rotation r, float k);

    glm::mat4 toMat4();

    void lookAt(glm::vec3 position, glm::vec3 target, glm::vec3 up);
};

class Transform {
public:
    glm::vec3 translation;
    glm::vec3 scaling;
    Rotation rotation;
public:
    Transform();
    Transform(Transform* other);

    void copy(Transform* other);

    glm::vec3 applyToPoint(glm::vec3 p);
    glm::vec3 applyToVector(glm::vec3 v);
    glm::vec3 applyToVersor(glm::vec3 n);

    Transform* combineWith(const Transform* t);

    Transform* inverse();

    Transform* mixWith(const Transform* t, float k);

    glm::mat4 toMat4();

    glm::mat4 toMat4NoScaling();

    glm::mat4 toMat4NoScalingNoRotation();

    glm::mat4 toNormal();

    glm::vec3 getRight();

    glm::vec3 getUp();

    glm::vec3 getForward();

    glm::vec3 worldToLocal(glm::vec3 point);



    void setLocalPosition(glm::vec3 position);

    void translate(glm::vec3 amount);

    void lookAt(const Transform* other);
    void lookAt(glm::vec3 target);
    void lookAtDirection(glm::vec3 forwardVector);
};

#endif