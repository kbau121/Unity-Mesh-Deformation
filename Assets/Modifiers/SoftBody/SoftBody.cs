using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class SoftBody : MonoBehaviour
{
    public float maxParticles = 8 * 8 * 8;
    public float minParticleDistance = 0.25f;

    public float springForce = 20f;
    public float dampingForce = 5f;

    Mesh deformingMesh;
    Vector3[] originalVertices, deformedVertices;
    Vector3[] vertexVelocities;

    Vector3[] particles;

    void Start()
    {
        deformingMesh = GetComponent<MeshFilter>().mesh;

        originalVertices = deformingMesh.vertices;
        deformedVertices = new Vector3[originalVertices.Length];
        for (int i = 0; i < originalVertices.Length; ++i)
        {
            deformedVertices[i] = originalVertices[i];
        }

        vertexVelocities = new Vector3[originalVertices.Length];

        // Poisson Disk Distribution for particle position
        List<Vector3> samples = new List<Vector3>();
        foreach (Vector3 sample in new PoissonDiscSampler(Vector3.one, 0.1f).Samples())
        {
            samples.Add(sample);
        }
        particles = samples.ToArray();
    }

    void Update()
    {
        
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.blue;
        foreach (Vector3 particle in particles)
        {
            Gizmos.DrawSphere(transform.TransformPoint(particle), 0.01f);
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
                Random.value * box.size.x,
                Random.value * box.size.y,
                Random.value * box.size.z));

            while (activeSamples.Count > 0)
            {
                int i = (int)Random.value * activeSamples.Count;
                Vector3 sample = activeSamples[i];

                bool found = false;
                for (int j = 0; j < k; ++j)
                {
                    Vector3 d = Random.onUnitSphere;
                    float r = Mathf.Sqrt(Random.value * 3 * radius2 + radius2);
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
