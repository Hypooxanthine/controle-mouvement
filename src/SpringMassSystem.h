#pragma once

#include "Box2D/Box2D.h"

#include <vector>

class SpringMassSystem
{
    struct Particle
    {
        // Masse de la particule (constante)
        float mass;
        // Constante de frottement (constante)
        float friction;

        // Position de la particule (variable au cours de la simulation)
        b2Vec2 position;
        // Vitesse de la particule (variable au cours de la simulation)
        b2Vec2 velocity;
        // Accélération de la particule (variable au cours de la simulation)
        b2Vec2 acceleration;
        // Somme des forces mises en jeu (variable au cours de la simulation)
        b2Vec2 forces;
    };

    struct Spring
    {
        // Longueur à vide (constante)
        float l0;
        // Constante de raideur (constante)
        float k;

        // Indices des particules auxquelles le ressort est lié
        size_t A, B;
    };

public:
    SpringMassSystem();

    // Ajoute une particule et renvoie son indice
    size_t addParticle(const Particle& p);
    // Ajoute un ressort et renvoie son indice
    size_t addSpring(const Spring& s);

    inline const std::vector<Particle>& getParticles() const { return m_Particles; }
    inline const std::vector<Spring>& getSprings() const { return m_Springs; }

    // Résolution des forces appliquées à chaque particule du système.
    // Le paramètre est une force constante/uniforme à ajouter à chaque particule (exemple : gravité)
    void solveForces(const b2Vec2& constantForce = b2Vec2(0.f, 0.f));

    void solvePositions(float dt);

    void solveCollisions(const std::vector<b2Body*>& bodies);

private:
    std::vector<Particle> m_Particles;
    std::vector<Spring> m_Springs;
};
