using UnityEngine;
public class InstantiationExample : MonoBehaviour {
    private Vector3 position; // object's position
    private Quaternion rotation; // object's rotation
    // Reference to the Prefab. Drag a Prefab into this field in the Inspector.
    public GameObject myPrefab;

    // This script will simply instantiate the Prefab when the game starts.
    void Start()
    {
        // Instantiate at position (0, 0, 0) and zero rotation.
        //Instantiate(myPrefab, new Vector3(0, 0, 0), Quaternion.identity);
    }

    void Update() {
        //position = new Vector3(Random.value, Random.value, Random.value);
        //rotation = new Quaternion(Random.value, Random.value, Random.value, Random.value);
        //myPrefab.transform.position = position;// = 10 * position;
        //Debug.Log(myPrefab.name);
        //Debug.Log(rotation);
    }
}