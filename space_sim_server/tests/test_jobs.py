import json
import time

from space_arm_platform.jobs import JobManager


def test_completed_episode_is_sealed_once_and_archived(tmp_path) -> None:
    episode_id = "episode-test"
    episode = tmp_path / "episodes" / episode_id
    episode.mkdir(parents=True)
    (episode / "metadata.json").write_text(
        json.dumps({"episode_id": episode_id, "status": "complete"}), encoding="utf-8"
    )
    (episode / "steps.jsonl").write_text("{}\n", encoding="utf-8")
    jobs = JobManager(tmp_path, workers=1)
    try:
        submitted = jobs.submit_archive(episode_id)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            result = jobs.get(submitted["job_id"])
            if result["status"] in {"completed", "failed"}:
                break
            time.sleep(0.01)
        assert result["status"] == "completed", result
        assert (episode / "artifact_manifest.json").is_file()
        assert (tmp_path / "archives" / f"{episode_id}.tar.gz").is_file()
        assert jobs.submit_archive(episode_id)["job_id"] == submitted["job_id"]
    finally:
        jobs.close()
