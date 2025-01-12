#include "SpringMassSystem.h"

SpringMassSystem::SpringMassSystem()
{
}

size_t SpringMassSystem::addParticle(const SpringMassSystem::Particle& p)
{
    m_Particles.push_back(p);
    return m_Particles.size() - 1;
}

size_t SpringMassSystem::addSpring(const SpringMassSystem::Spring& s)
{
    m_Springs.push_back(s);
    return m_Springs.size() - 1;
}

void SpringMassSystem::solveForces(const b2Vec2& constantForce)
{
    // Première étape : remise à zéro puis ajout de la force constante
    for (Particle& p : m_Particles)
    {
        p.forces = constantForce;
    }

    for (Spring& s : m_Springs)
    {
        Particle& A = m_Particles[s.A];
        Particle& B = m_Particles[s.B];

        b2Vec2 AB = B.position - A.position;
        b2Vec2 nAB = AB;
        float l = nAB.Normalize(); // Longueur actuelle du ressort

        // Force de rappel
        b2Vec2 F = s.k * (l - s.l0) * nAB;

        // Ajout des forces aux particules
        A.forces += F;
        B.forces -= F;

        // Ajout des forces de frottement
        A.forces -= A.friction * A.velocity;
        B.forces -= B.friction * B.velocity;
    }
}

void SpringMassSystem::solvePositions(float dt)
{
    for (Particle& p : m_Particles)
    {
        // Calcul de l'accélération
        p.acceleration = { p.forces.x / p.mass, p.forces.y / p.mass };

        // Intégration de la vitesse
        p.velocity += { p.acceleration.x * dt, p.acceleration.y * dt };

        // Intégration de la position
        p.position += { p.velocity.x * dt, p.velocity.y * dt };
    }
}

static bool IsPointInsideBody(b2Body* body, const b2Vec2& p)
{
    for (b2Fixture* fixture = body->GetFixtureList(); fixture; fixture = fixture->GetNext())
    {
        if (fixture->TestPoint(p))
            return true;
    }

    return false;
}

void SpringMassSystem::solveCollisions(const std::vector<b2Body*>& bodies)
{
    for (Particle& p : m_Particles)
    {
        for (b2Body* body : bodies)
        {
            if (IsPointInsideBody(body, p.position))
            {
                // On annule la vitesse dans la direction de la normale
                b2Vec2 normal = p.position - body->GetWorldCenter();
                normal.Normalize();
                p.velocity -= 1.1f * b2Dot(p.velocity, normal) * normal;
            }
        }
    }
}