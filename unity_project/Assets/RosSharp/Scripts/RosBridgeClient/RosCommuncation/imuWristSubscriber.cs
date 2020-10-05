using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class imuWristSubscriber : Subscriber<Messages.Geometry.Pose>
    {
        //public Transform PublishedTransform;

        public Vector3 position;
        public Quaternion rotation;
        public bool isMessageReceived;
        private RosConnector ros;
        protected override void Start()
        {
			base.Start();
		}
		
        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }
	void Awake() {
	    player = GameObject.FindObjectOfType<Player> ();
	}

        protected override void ReceiveMessage(Messages.Geometry.Pose message)
        {
            position = GetPosition(message).Ros2Unity();
            rotation = GetRotation(message).Ros2Unity();
	    Vector3 euler = rotation.eulerAngles;
	    transform.rotation = rotation;//new Vector3(euler.x, euler.y, euler.z);
	    //Debug.Log(euler);
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            //PublishedTransform.position = position;
            //PublishedTransform.rotation = rotation;
        }

        private Vector3 GetPosition(Messages.Geometry.Pose message)
        {
            return new Vector3(
                message.position.x,
                message.position.y,
                message.position.z);
        }

        private Quaternion GetRotation(Messages.Geometry.Pose message)
        {
            return new Quaternion(
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w);
        }
    }
}
