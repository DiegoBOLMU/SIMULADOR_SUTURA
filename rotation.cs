


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OrbitAroundSphere : MonoBehaviour
{
    public Transform targetSphere;  // El objeto esfera alrededor del cual quieres orbitar.
    public float orbitSpeed = 30.0f;  // Velocidad de la órbita.

    private Vector3 axisOfRotation = Vector3.forward;  // Eje de rotación, en este caso, el eje Z.
    private bool isOrbiting = false;  // Variable de control para la órbita.

    private float moveSpeed = 0.1f; // Velocidad de movimiento en X.
    private bool isMoving = false; // Variable de control para el movimiento en X.
    private Vector3 initialPosition; // Posición inicial del objeto.

    void Start()
    {
        initialPosition = transform.position; // Guarda la posición inicial del objeto.
    }

    void Update()
    {
        // Comienza o detiene la órbita al presionar la tecla "R".
        if (Input.GetKeyDown(KeyCode.R))
        {
            isOrbiting = !isOrbiting;
        }

        // Asegúrate de que el objeto de la esfera esté asignado en el Inspector.
        if (targetSphere == null)
        {
            Debug.LogError("El objeto de la esfera no está asignado.");
            return;
        }

        // Si no estamos en órbita, permite el movimiento en X.
        if (!isOrbiting)
        {
            if (Input.GetKey(KeyCode.W))
            {
                MoveObjectX(moveSpeed);
            }
            else if (Input.GetKey(KeyCode.S) && transform.position.y > initialPosition.y)
            {
                MoveObjectX(-moveSpeed);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha2)) // Agregado: Mover en el eje Z.
            {
                MoveObjectZ(-20.0f);
            }
        }

        // Calcula la rotación si estamos en órbita.
        if (isOrbiting)
        {
            transform.RotateAround(targetSphere.position, axisOfRotation, orbitSpeed * Time.deltaTime);
        }
    }

    void MoveObjectX(float distance)
    {
        // Mueve el objeto en la dirección de X de manera gradual.
        transform.Translate(Vector3.right * distance);
    }

    // Nuevo método para mover en el eje Z.
    void MoveObjectZ(float distance)
    {
        // Mueve el objeto en la dirección de Z de manera gradual.
        transform.Translate(Vector3.forward * distance);
    }
}