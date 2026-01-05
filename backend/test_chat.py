"""
Test script to verify chat history functionality
"""
import asyncio
import uuid
import httpx


async def test_chat_history():
    """Test the complete chat history flow"""

    # Generate a new session ID
    session_id = str(uuid.uuid4())
    print(f"Testing with session ID: {session_id}")
    print("=" * 60)
    print()

    api_url = "http://localhost:8000"

    async with httpx.AsyncClient(timeout=60.0) as client:
        # Test 1: Send first message
        print("Test 1: Sending first message...")
        response1 = await client.post(
            f"{api_url}/api/chat",
            json={
                "message": "What is ROS2?",
                "session_id": session_id
            }
        )

        if response1.status_code == 200:
            data1 = response1.json()
            print(f"[OK] Response received (latency: {data1['latency']}s)")
            print(f"   Session ID: {data1['session_id']}")
            print(f"   Response preview: {data1['response'][:100]}...")
            print(f"   Sources: {len(data1['sources'])} found")
        else:
            print(f"[ERROR] Error: {response1.status_code} - {response1.text}")
            return

        print()

        # Test 2: Send second message to same session
        print("Test 2: Sending second message to same session...")
        response2 = await client.post(
            f"{api_url}/api/chat",
            json={
                "message": "Tell me about nodes",
                "session_id": session_id
            }
        )

        if response2.status_code == 200:
            data2 = response2.json()
            print(f"[OK] Response received (latency: {data2['latency']}s)")
            print(f"   Response preview: {data2['response'][:100]}...")
        else:
            print(f"[ERROR] Error: {response2.status_code}")
            return

        print()

        # Test 3: Retrieve chat history
        print("Test 3: Retrieving chat history...")
        response3 = await client.get(
            f"{api_url}/api/chat/history/{session_id}"
        )

        if response3.status_code == 200:
            history = response3.json()
            print(f"[OK] History retrieved successfully")
            print(f"   Session ID: {history['session_id']}")
            print(f"   Total messages: {len(history['messages'])}")
            print()
            print("   Message history:")
            for i, msg in enumerate(history['messages'], 1):
                role = "User" if msg['role'] == 'user' else "Bot"
                content_preview = msg['content'][:60] + "..." if len(msg['content']) > 60 else msg['content']
                print(f"   {i}. [{role}] {content_preview}")

            # Verify we have both user and assistant messages
            user_msgs = [m for m in history['messages'] if m['role'] == 'user']
            assistant_msgs = [m for m in history['messages'] if m['role'] == 'assistant']

            print()
            if len(user_msgs) == 2 and len(assistant_msgs) == 2:
                print("[OK] CHAT HISTORY TEST PASSED!")
                print(f"   - {len(user_msgs)} user messages saved")
                print(f"   - {len(assistant_msgs)} assistant messages saved")
                print("   - All messages persist correctly")
            else:
                print(f"[WARN]  Unexpected message count:")
                print(f"   - User messages: {len(user_msgs)} (expected 2)")
                print(f"   - Assistant messages: {len(assistant_msgs)} (expected 2)")

        elif response3.status_code == 404:
            print(f"[ERROR] Session not found in database")
            print("   This means messages were not saved properly")
        else:
            print(f"[ERROR] Error: {response3.status_code} - {response3.text}")
            return

    print()
    print("=" * 60)
    print("Test complete!")
    print()
    print(f"You can verify in browser by:")
    print(f"1. Opening localStorage in DevTools")
    print(f"2. Setting: chatbot_session_id = {session_id}")
    print(f"3. Refreshing and opening chatbot")
    print(f"4. You should see the 2 messages above")


if __name__ == "__main__":
    print()
    print("Testing Chat History Functionality")
    print()
    asyncio.run(test_chat_history())
