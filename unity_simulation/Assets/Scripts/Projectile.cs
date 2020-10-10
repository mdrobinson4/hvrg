using UnityEngine;
public class Projectile : MonoBehaviour
{
   public GameObject explosion;
   void OnCollisionEnter()
   {
       Instantiate(explosion,transform.position,transform.rotation);
       Destroy(gameObject);
   }
}