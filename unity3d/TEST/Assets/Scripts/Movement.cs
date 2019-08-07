//using System.Collections;
//using System.Collections.Generic;
//using UnityEditor.VersionControl;
using UnityEngine;

public class Movement : MonoBehaviour
{
    public Material[] material;
    private Renderer rend;
    
    public float mSpeed;
    private float ray_target_speed;
    public bool picked;
    public bool didHit;

    public bool mouse_move_complete;

    private Vector3 targetPosition;
    
    // Start is called before the first frame update
    void Start()
    {
        rend = GetComponent<Renderer>();
        rend.enabled = true;
        rend.sharedMaterial = material[0];

        mSpeed = 7f;
        ray_target_speed = 20f;
        mouse_move_complete = false;
    }

    // Update is called once per frame
    void Update()
    {
        
        if (Input.GetMouseButtonDown(0))
        {
            Ray toMouse = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit rhInfo;
            didHit = Physics.Raycast(toMouse, out rhInfo, 500.0f);
            if (didHit)
            {
                if (rhInfo.collider.name == "Cube")
                {
                    if (picked)
                    {
                        rend.sharedMaterial = material[0];
                        picked = false;
                    }
                    else if (!picked)
                    {
                        rend.sharedMaterial = material[1];
                        picked = true;
                    }
                }
                else if(rhInfo.collider.name == "cube_background")
                {
                    // Move this cube's position to clicked x, y coordinates if the transparent collider was clicked,,   
                    targetPosition = rhInfo.point;
                    targetPosition.z = transform.position.z;

                    mouse_move_complete = false;

                }
                

                Debug.Log(rhInfo.collider.name + "    " + rhInfo.point);

            }
        }

        
        if (mouse_move_complete == false)
        {
            mouse_Move(); 
        }

        else if (mouse_move_complete)
        {
            if (picked)
            {
                transform.Translate(mSpeed * Input.GetAxis("Horizontal") * Time.deltaTime, 
                                    0f,
                                    mSpeed * Input.GetAxis("Vertical") * Time.deltaTime);
            }
        }
    }

    void mouse_Move()
    {
        if (transform.position != targetPosition)
        {
            transform.position =
                Vector3.MoveTowards(transform.position, targetPosition, ray_target_speed * Time.deltaTime);
        }
        else
        {
            mouse_move_complete = true;
        }

    }
}
