"""HTTP authentication, user management and shared access decisions."""
from __future__ import annotations

import asyncio
from typing import Any

from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse

from .auth import AuthError, SESSION_COOKIE
from .models import LoginRequest, OperatorCreate, PasswordChange, PasswordReset, UserCreate, UserUpdate


def current_user(request: Request) -> dict[str, Any]:
    user = getattr(request.state, "user", None)
    if not user:
        raise HTTPException(status_code=401, detail="请先登录")
    return user


def require_admin(request: Request) -> dict[str, Any]:
    user = current_user(request)
    if user["role"] != "admin":
        raise HTTPException(status_code=403, detail="仅管理员可以管理操作员")
    return user


def can_manage_scene(user: dict[str, Any], status: dict[str, Any]) -> bool:
    owner = (status.get("instance") or {}).get("created_by") or {}
    return user["role"] == "admin" or owner.get("user_id") == user["user_id"]


def install_auth_routes(app, *, config, auth, allowed_origins, login_limiter):
    @app.middleware("http")
    async def authenticated_api(request: Request, call_next):
        origin = request.headers.get("origin")
        if allowed_origins and (
            (origin is not None and origin not in allowed_origins)
            or (request.method not in {"GET", "HEAD", "OPTIONS"} and origin is None)
        ):
            return JSONResponse({"detail": "origin not allowed"}, status_code=403, headers={"Cache-Control": "no-store"})
        path = request.url.path
        if path.startswith("/api/") and path not in {"/api/health", "/api/auth/login"}:
            user = auth.session_user(request.cookies.get(SESSION_COOKIE))
            if not user:
                return JSONResponse({"detail": "请先登录"}, status_code=401, headers={"Cache-Control": "no-store"})
            request.state.user = user
        response = await call_next(request)
        if path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-store"
        return response

    @app.post("/api/auth/login")
    async def login(payload: LoginRequest, request: Request) -> JSONResponse:
        retry_after = login_limiter.retry_after(request.client.host if request.client else "unknown")
        if retry_after:
            return JSONResponse(
                {"detail": "登录尝试过于频繁，请稍后重试"}, status_code=429,
                headers={"Retry-After": str(retry_after)},
            )
        user = await asyncio.to_thread(auth.authenticate, payload.username, payload.password)
        if not user:
            raise HTTPException(status_code=401, detail="用户名或密码错误")
        try:
            token, max_age = await asyncio.to_thread(auth.create_session, user["user_id"])
        except AuthError as error:
            raise HTTPException(status_code=401, detail="账号不可用") from error
        response = JSONResponse({"user": user})
        response.set_cookie(
            SESSION_COOKIE, token, max_age=max_age, httponly=True, samesite="strict",
            secure=config.secure_cookies, path="/",
        )
        return response

    @app.get("/api/auth/me")
    async def auth_me(request: Request) -> dict[str, Any]:
        return {"user": current_user(request)}

    @app.post("/api/auth/logout")
    async def logout(request: Request) -> JSONResponse:
        await asyncio.to_thread(auth.delete_session, request.cookies.get(SESSION_COOKIE))
        response = JSONResponse({"ok": True})
        response.delete_cookie(SESSION_COOKIE, path="/", secure=config.secure_cookies, httponly=True, samesite="strict")
        return response

    @app.post("/api/auth/change-password")
    async def change_password(payload: PasswordChange, request: Request) -> JSONResponse:
        user = current_user(request)
        try:
            await asyncio.to_thread(
                auth.change_password, user["user_id"], payload.current_password, payload.new_password
            )
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        response = JSONResponse({"ok": True})
        response.delete_cookie(SESSION_COOKIE, path="/", secure=config.secure_cookies, httponly=True, samesite="strict")
        return response

    @app.get("/api/users")
    async def list_users(request: Request) -> dict[str, Any]:
        require_admin(request)
        return {"users": await asyncio.to_thread(auth.list_users)}

    @app.post("/api/users/operators")
    async def create_operator(payload: OperatorCreate, request: Request) -> dict[str, Any]:
        require_admin(request)
        try:
            return await asyncio.to_thread(auth.create_operator, payload.username, payload.password)
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.post("/api/users", status_code=201)
    async def create_user(payload: UserCreate, request: Request) -> dict[str, Any]:
        require_admin(request)
        try:
            return await asyncio.to_thread(auth.create_user, **payload.model_dump())
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.patch("/api/users/{user_id}")
    async def update_user(user_id: str, payload: UserUpdate, request: Request) -> dict[str, Any]:
        actor = require_admin(request)
        try:
            return await asyncio.to_thread(auth.update_user, actor["user_id"], user_id, **payload.model_dump())
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.post("/api/users/{user_id}/reset-password")
    async def reset_user_password(user_id: str, payload: PasswordReset, request: Request) -> dict[str, Any]:
        actor = require_admin(request)
        try:
            await asyncio.to_thread(auth.reset_user_password, actor["user_id"], user_id, payload.password)
            return {"ok": True}
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.delete("/api/users/operators/{user_id}")
    async def delete_operator(user_id: str, request: Request) -> dict[str, Any]:
        require_admin(request)
        try:
            await asyncio.to_thread(auth.delete_operator, user_id)
            return {"ok": True}
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.post("/api/users/operators/{user_id}/reset-password")
    async def reset_operator_password(user_id: str, payload: PasswordReset, request: Request) -> dict[str, Any]:
        require_admin(request)
        try:
            await asyncio.to_thread(auth.reset_operator_password, user_id, payload.password)
            return {"ok": True}
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
