using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SoftBodyInput : MonoBehaviour
{
    private Rigidbody selected;
    private Plane dragPlane;

    private float forceScale = 1f;
    private float scrollScale = 0.1f;

    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Select();
        }
        else if (Input.GetMouseButtonUp(0))
        {
            Deselect();
        }

        if (Input.mouseScrollDelta.y != 0)
        {
            forceScale *= Mathf.Pow(10, Input.mouseScrollDelta.y * scrollScale);
        }
    }

    private void FixedUpdate()
    {
        if (selected)
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            float enter;

            if (dragPlane.Raycast(ray, out enter))
            {
                Vector3 hit = ray.GetPoint(enter);
                Vector3 offset = hit - selected.position;

                selected.AddForce(forceScale * offset, ForceMode.Impulse);
            }
        }
    }

    private void Select()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        int layerMask = 1 << LayerMask.NameToLayer("Soft Body");

        if (Physics.Raycast(ray, out hit, 1000f, layerMask))
        {
            SoftBody softBody = hit.collider.GetComponentInParent<SoftBody>();

            if (softBody)
            {
                Rigidbody rigidbody = hit.collider.GetComponent<Rigidbody>();

                if (rigidbody)
                {
                    dragPlane = new Plane(ray.direction, hit.point);
                    selected = rigidbody;
                }
            }
        }
    }

    private void Deselect() {
        selected = null;
    }
}
