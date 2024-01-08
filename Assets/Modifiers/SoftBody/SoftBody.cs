using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class SoftBody : MonoBehaviour
{
    public float minParticleDistance = 0.25f;

    public float springConstant = 5f;
    public float dampingConstant = 0.025f;

    public Vector3 size = Vector3.one;

    Mesh deformingMesh;
    Vector3[] originalVertices, deformedVertices;

    Particle[] particles = new Particle[0];
    Spring[] springs = new Spring[0];

    private void Awake()
    {
        deformingMesh = GetComponent<MeshFilter>().mesh;

        originalVertices = deformingMesh.vertices;
        deformedVertices = new Vector3[originalVertices.Length];
        for (int i = 0; i < originalVertices.Length; ++i)
        {
            deformedVertices[i] = originalVertices[i];
        }

        InitParticles();
        InitSprings();
        BindMesh();
    }

    private void Start()
    {

    }

    private void Update()
    {
        for (int i = 0; i < originalVertices.Length; ++i)
        {
            Vector3 newVertex = Vector3.zero;

            foreach (Particle particle in particles)
            {
                particle.UpdateVertex(ref newVertex, i, ref originalVertices);
            }

            deformedVertices[i] = newVertex;
        }

        deformingMesh.vertices = deformedVertices;
        deformingMesh.RecalculateNormals();
    }

    private void FixedUpdate()
    {
        foreach (Spring spring in springs)
        {
            spring.ApplyImpulse(ref particles);
        }

        foreach (Particle particle in particles)
        {
            particle.Impulse(dampingConstant * -particle.velocity);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.blue;
        foreach (Particle particle in particles)
        {
            Gizmos.DrawSphere(particle.position, particle.radius / 5f);
        }

        Gizmos.color = Color.green;
        foreach (Spring spring in springs)
        {
            Gizmos.DrawLine(particles[spring.i].position, particles[spring.j].position);
        }
    }

    private void InitParticles()
    {
        float radius = minParticleDistance * (0.5f * 0.95f);

        // Poisson Disk Distribution for particle position
        List<Particle> particleList = new List<Particle>();
        foreach (Vector3 sample in new PoissonDiscSampler(size, minParticleDistance).Samples())
        {
            particleList.Add(new Particle(sample, radius, transform));
        }

        for (int x = 0; x <= 1; ++x)
        {
            for (int y = 0; y <= 1; ++y)
            {
                for (int z = 0; z <= 1; ++z)
                {
                    particleList.Add(new Particle(new Vector3(x, y, z), radius, transform));
                }
            }
        }

        particles = particleList.ToArray();
    }

    private void InitSprings()
    {
        Spring.k = springConstant;
        int springPerParticle = 12;

        List<Spring> springList = new List<Spring>();
        for (int i = 0; i < particles.Length; i++)
        {
            int[] closestParticles = FindClosestParticles(springPerParticle, particles[i].position, i);

            foreach (int closestParticle in closestParticles)
            {
                Spring nextSpring = new Spring(i, closestParticle, ref particles);
                if (!springList.Contains(nextSpring))
                {
                    springList.Add(nextSpring);
                }

                if ((particles[i].position - particles[closestParticle].position).magnitude <= particles[i].radius + particles[closestParticle].radius)
                {
                    Physics.IgnoreCollision(particles[i].collider, particles[closestParticle].collider);
                }
            }
        }

        springs = springList.ToArray();
    }

    private void BindMesh()
    {
        List<KeyValuePair<int, float>>[] weights = new List<KeyValuePair<int, float>>[particles.Length];
        for (int i = 0; i < weights.Length; ++i)
        {
            weights[i] = new List<KeyValuePair<int, float>>();
        }

        for (int i = 0; i < originalVertices.Length; ++i)
        {
            int[] closestParticles = FindClosestParticles(1, originalVertices[i]);

            weights[closestParticles[0]].Add(new KeyValuePair<int, float>(i, 1f));
        }

        for (int i = 0; i < particles.Length; ++i) {
            particles[i].BindVertices(weights[i].ToArray());
        }
    }

    private int[] FindClosestParticles(int k, Vector3 target, int targetParticle = -1)
    {
        if (particles.Length < k) k = particles.Length;

        int[] closestParticles = new int[k];
        float[] closestParticleDistances = new float[k];
        for (int i = 0; i < closestParticles.Length; ++i)
        {
            closestParticleDistances[i] = float.PositiveInfinity;
        }

        for (int i = 0; i < particles.Length; ++i)
        {
            if (targetParticle == i) continue;

            float distance = (particles[i].position - target).sqrMagnitude;

            if (distance < closestParticleDistances[k - 1])
            {
                closestParticles[k - 1] = i;
                closestParticleDistances[k - 1] = distance;

                for (int j = k - 2; j >= 0; --j)
                {
                    if (distance < closestParticleDistances[j])
                    {
                        closestParticles[j + 1] = closestParticles[j];
                        closestParticleDistances[j + 1] = closestParticleDistances[j];

                        closestParticles[j] = i;
                        closestParticleDistances[j] = distance;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }

        return closestParticles;
    }

    private class Particle
    {
        private static int _ID;

        public Vector3 position {
            get { return _gameObject.transform.position; }
            set { _gameObject.transform.position = value; }
        }

        public Vector3 velocity
        {
            get { return _rigidbody.velocity; }
            set { _rigidbody.velocity = value; }
        }

        public float radius
        {
            get { return _collider.radius; }
            set { _collider.radius = value; }
        }

        public SphereCollider collider
        {
            get { return _collider; }
            private set { _collider = value; }
        }

        private GameObject _gameObject;
        private Rigidbody _rigidbody;
        private SphereCollider _collider;

        private Dictionary<int, float> _boundWeights;
        private Vector3 _boundTranslation;

        public Particle() : this(Vector3.zero, 0f) { }

        public Particle(Vector3 position, float radius, Transform parent = null)
        {
            _gameObject = new GameObject(String.Format("Particle{0}", _ID++));
            _rigidbody = _gameObject.AddComponent<Rigidbody>();
            _collider = _gameObject.AddComponent<SphereCollider>();
            _boundWeights = new Dictionary<int, float>();

            _gameObject.transform.position = position;
            if (parent)
            {
                _gameObject.transform.parent = parent;
            }

            _rigidbody.constraints |= RigidbodyConstraints.FreezeRotation;

            _collider.radius = radius;
        }

        public void Impulse(Vector3 force)
        {
            _rigidbody.AddForce(force, ForceMode.Impulse);
        }

        public void UpdateVertex(ref Vector3 vertex, int index, ref Vector3[] originalVertices)
        {
            if (!_boundWeights.ContainsKey(index)) return;

            // sum(w * LTW * B) * orig
            vertex += _boundWeights[index] * (_gameObject.transform.position - _boundTranslation) + originalVertices[index];
        }

        public void BindVertices(KeyValuePair<int, float>[] weights)
        {
            _boundWeights = new Dictionary<int, float>(weights);
            _boundTranslation = _gameObject.transform.position;
        }
    }

    private class Spring
    {
        public static float k = 1f;

        public int i, j;
        public float length;

        public Spring(int i, int j, ref Particle[] particles)
            : this(i, j, (particles[i].position - particles[j].position).magnitude)
        { }

        public Spring(int i, int j, float length)
        {
            this.i = i;
            this.j = j;
            this.length = length;
        }

        public void ApplyImpulse(ref Particle[] particles)
        {
            Vector3 offset = particles[j].position - particles[i].position;
            Vector3 springImpulse = offset.normalized * (offset.magnitude - length) * k;

            particles[i].Impulse(springImpulse);
            particles[j].Impulse(-springImpulse);
        }

        public override bool Equals(object obj)
        {
            if ((obj == null) || !this.GetType().Equals(obj.GetType())) return false;

            Spring other = (Spring)obj;

            bool sameParticles =
                (other.i == i && other.j == j) ||
                (other.i == j && other.j == i);

            return sameParticles && other.length == length;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }

    // Paper Reference: https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
    // Source Reference: http://gregschlom.com/devlog/2014/06/29/Poisson-disc-sampling-Unity.html
    private class PoissonDiscSampler
    {
        private const int k = 30;

        private readonly Box box;
        private readonly float radius2;
        private readonly float cellSize;
        private Vector3[,,] grid;
        private List<Vector3> activeSamples = new List<Vector3>();

        public PoissonDiscSampler(Vector3 size, float radius)
        {
            box = new Box(Vector3.zero, size);
            radius2 = radius * radius;
            cellSize = radius / Mathf.Sqrt(2);
            grid = new Vector3[
                Mathf.CeilToInt(size.x / cellSize),
                Mathf.CeilToInt(size.y / cellSize),
                Mathf.CeilToInt(size.z / cellSize)
                ];
        }

        public IEnumerable<Vector3> Samples()
        {
            yield return AddSample(new Vector3(
                UnityEngine.Random.value * box.size.x,
                UnityEngine.Random.value * box.size.y,
                UnityEngine.Random.value * box.size.z));

            while (activeSamples.Count > 0)
            {
                int i = (int)UnityEngine.Random.value * activeSamples.Count;
                Vector3 sample = activeSamples[i];

                bool found = false;
                for (int j = 0; j < k; ++j)
                {
                    Vector3 d = UnityEngine.Random.onUnitSphere;
                    float r = Mathf.Sqrt(UnityEngine.Random.value * 3 * radius2 + radius2);
                    Vector3 candidate = sample + r * d;

                    if (box.contains(candidate) && IsFarEnough(candidate))
                    {
                        found = true;
                        yield return AddSample(candidate);
                        break;
                    }
                }

                if (!found)
                {
                    activeSamples.RemoveAt(i);
                }
            }
        }

        private bool IsFarEnough(Vector3 sample)
        {
            Vector3Int gridPosition = sampleToGridPosition(sample, cellSize);
            Vector3Int gridSize = new Vector3Int(grid.GetLength(0), grid.GetLength(1), grid.GetLength(2));

            Vector3Int min = Vector3Int.Max(gridPosition - new Vector3Int(2, 2, 2), Vector3Int.zero);
            Vector3Int max = Vector3Int.Min(gridPosition + new Vector3Int(2, 2, 2), gridSize - Vector3Int.one);

            for (int z = min.z; z <= max.z; ++z)
            {
                for (int y = min.y; y <= max.y; ++y)
                {
                    for (int x = min.x; x <= max.x; ++x)
                    {
                        Vector3 s = grid[x, y, z];
                        if (s != Vector3.zero)
                        {
                            Vector3 d = s - sample;
                            if (d.sqrMagnitude < radius2)
                                return false;
                        }
                    }
                }
            }

            return true;
        }

        private Vector3 AddSample(Vector3 sample)
        {
            activeSamples.Add(sample);
            Vector3Int gridPosition = sampleToGridPosition(sample, cellSize);
            grid[gridPosition.x, gridPosition.y, gridPosition.z] = sample;
            return sample;
        }

        private Vector3Int sampleToGridPosition(Vector3 sample, float cellSize)
        {
            return new Vector3Int(
                (int)(sample.x / cellSize),
                (int)(sample.y / cellSize),
                (int)(sample.z / cellSize)
                );
        }

        private struct Box
        {
            public Vector3 position;
            public Vector3 size;

            public Box(Vector3 position, Vector3 size)
            {
                this.position = position;
                this.size = size;
            }

            public bool contains(Vector3 position)
            {
                return (
                    position.x > this.position.x &&
                    position.y > this.position.y &&
                    position.z > this.position.z
                    ) && (
                    position.x < this.position.x + size.x &&
                    position.y < this.position.y + size.y &&
                    position.z < this.position.z + size.z
                    );
            }
        }
    }
}
