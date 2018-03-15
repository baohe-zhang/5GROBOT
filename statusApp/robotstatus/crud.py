from robotstatus import get_model
from flask import Blueprint, redirect, render_template, request, url_for
import json

crud = Blueprint('crud', __name__)

@crud.route("/")
def index():
    robot1, robot2, robot3, robot4 = get_model().retrieve_status()
    return render_template('index.html', robot1=robot1, robot2=robot2, robot3=robot3, robot4=robot4)


@crud.route("/<robot_id>")
def detail(robot_id):
    messages = get_model().retrieve(robot_id)
    return render_template('detail.html', messages=messages)


@crud.route('/add', methods=['GET', 'POST'])
def add():
    if request.method == 'POST':
        data = request.get_json()   # data is a dict
        message = get_model().create(data)
        return 'ADD'
    else:
        return 'NO JSON'
        

