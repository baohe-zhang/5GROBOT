from flask import Flask
from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()

def init_app(app):
    # Disable track modifications, as it unnecessarily uses memory.
    app.config.setdefault('SQLALCHEMY_TRACK_MODIFICATIONS', False)
    db.init_app(app)


def from_sql(row):
    """Translates a SQLAlchemy model instance into a dictionary"""
    data = row.__dict__.copy()
    data['id'] = row.id
    data.pop('_sa_instance_state')
    return data

# [START model]
class Message(db.Model):
    __tablename__ = 'messages'

    id = db.Column(db.Integer, primary_key=True)
    robotId = db.Column(db.String(255))
    ipAddress = db.Column(db.String(255))
    command = db.Column(db.String(255))
    speed = db.Column(db.String(255))
    angle = db.Column(db.String(255))
    trackStatus = db.Column(db.String(255))
    detectStatus = db.Column(db.String(255))
    time = db.Column(db.String(255))

    def __repr__(self):
        return str(self.id) + "<Message(robotId=%s, ipAddress=%s, command=%s, speed=%s, angle=%s, trackStatus=%s, detectStatus=%s, time=%s)" % (self.robotId, self.ipAddress, self.command, self.speed, self.angle, self.trackStatus, self.detectStatus, self.time)
# [END model]


def retrieve_status():
    robot1 = Message.query.filter_by(robotId='robot1').order_by(Message.id.desc()).first()
    robot2 = Message.query.filter_by(robotId='robot2').order_by(Message.id.desc()).first()
    robot3 = Message.query.filter_by(robotId='robot3').order_by(Message.id.desc()).first()
    robot4 = Message.query.filter_by(robotId='robot4').order_by(Message.id.desc()).first()

    return robot1, robot2, robot3, robot4


def retrieve(robot_id):
    messages = Message.query.filter_by(robotId=robot_id).order_by(Message.id.desc()).all()

    return messages


def create(data):
    print(str(data))
    message = Message(**data)
    db.session.add(message)
    db.session.commit()
    print('message created')
    print(message)
    
    return None


def _create_database():
    """
    If this script is run directly, create all the tables necessary to run the
    application.
    """
    app = Flask(__name__)
    app.config.from_pyfile('../config.py')
    init_app(app)
    with app.app_context():
        db.create_all()
    print("All tables created")


if __name__ == '__main__':
    _create_database()
