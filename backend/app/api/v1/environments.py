from fastapi import APIRouter

router = APIRouter(prefix="/environments", tags=["environments"])

ENVIRONMENTS = [
    {
        "id": "open_space",
        "name": "Open Space",
        "description": "Minimal environment with ground plane. Ideal for basic manipulation testing.",
    },
    {
        "id": "kitchen",
        "name": "Kitchen",
        "description": "Kitchen countertop with table. Suited for household object manipulation.",
    },
    {
        "id": "workshop",
        "name": "Workshop",
        "description": "Industrial workbench setup. For tools and mechanical parts.",
    },
    {
        "id": "vehicle",
        "name": "Vehicle Interior",
        "description": "Dashboard surface. For automotive component testing.",
    },
    {
        "id": "operating_room",
        "name": "Operating Room",
        "description": "Sterile surgical table. For medical device manipulation.",
    },
]


@router.get("")
async def list_environments():
    return ENVIRONMENTS
