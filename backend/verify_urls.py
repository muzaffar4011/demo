"""
Quick script to verify that URLs in Qdrant are correctly formatted.
"""
import asyncio
from app.core.database import get_qdrant_client
from app.core.config import settings


async def main():
    print("Verifying URLs in Qdrant collection...")
    print()

    qdrant_client = get_qdrant_client()

    # Get a sample of points from the collection
    try:
        # Scroll through first 10 points
        result = await qdrant_client.scroll(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            limit=10,
            with_payload=True
        )

        points = result[0]  # First element is the list of points

        print(f"Sample URLs from the collection:")
        print("=" * 80)

        seen_urls = set()
        for point in points:
            if point.payload and 'url' in point.payload:
                url = point.payload['url']
                title = point.payload.get('title', 'N/A')

                # Only show unique URLs
                if url not in seen_urls:
                    seen_urls.add(url)
                    print(f"Title: {title}")
                    print(f"URL:   {url}")
                    print()

        print("=" * 80)
        print(f"Total unique URLs shown: {len(seen_urls)}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
